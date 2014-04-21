from Tkinter import *
import ttk
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

  def CloseWindow(self):
      # self.communicator.stop()
      self.quit()

def main():
  
  root = Tk()
  root.geometry(str(window_width)+"x"+str(window_height))
  app = App(root)
  root.mainloop()  


if __name__ == '__main__':
  main()  