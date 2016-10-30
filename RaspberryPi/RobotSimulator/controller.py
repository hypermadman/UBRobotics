from Tkinter import *
from eurobot import *
from pathPrint import pathPrint
import numpy
import tkFont
import time
import thread

class controller(object):

    def setRecord(self): #Move robot to co-ordinate
        #First orient the robot to the destination	
        dx = self.firstPoint[0] - self.primaryRobot.position[0]
	dy = self.firstPoint[1] - self.primaryRobot.position[1]
	
	if abs(dx) + abs(dy) > 0: #As long as there is some translation
                if self.recording == True:
                    self.w.create_line(self.firstPoint[0],self.firstPoint[1],self.primaryRobot.position[0],self.primaryRobot.position[1],width = 2, fill = "green") #Draw path line
		
                angle = math.atan2(dx,-dy) #Return signed tangent angle from robot to destination RELATIVE TO EAST
		north = -(math.degrees(angle)) #Correct to North, change to degrees
		rotate = north - self.primaryRobot.angle #Angular displacement
                direction = 1 #initially forward

                if self.reversegear.get() == 1: #reversing, so bring it back another 180
                    rotate += 180
                    direction = -1 #reverse

                self.primaryRobot.rotate(rotate) #Rotate the robot
                distance = int(numpy.linalg.norm([dy,dx]))
                self.primaryRobot.translate(distance,self.reversegear.get()) #Translate the robot
                text1 = str(int(self.primaryRobot.rotation))
                text2 = str(direction*int(distance/self.pixeltoCM[0]))

                if self.recording == True:
                    self.printer = self.printer + "\nrobot.rotate(" + text1 + ");" #Get calibrated rotational displacement
                    self.printer = self.printer + "\nrobot.move(" + text2 + ");" #Multiply by direction to make move negative or positive depending on direction of travel	
                    #print printer
 
    def setPrimary(self): #function to 'build' primary robot according to text input
        self.primaryRobot = Eurobot(int(eval(self.e8.get())*self.pixeltoCM[1]),int(eval(self.e9.get())*self.pixeltoCM[0]),[int(self.e5.get()),int(self.e6.get())],int(self.e7.get()),self.w) #set parameters according to input fields

    def flipPrimary(self): #flip primary robot to other side of arena
        reflect = self.datum[0] - self.boundsize[0]/2 #X co-ordinate of line of reflection
        translation = reflect - self.primaryRobot.position[0] #X displacement of robot from line of reflection
        self.primaryRobot.abstranslate([2*translation,0]) #Translate without taking in account angle of robot
        self.primaryRobot.rotate(180)
        #print self.pixeltoCM
       
    def setPath(self):
        if self.recording == True:
            if self.printer != "":
                self.printed = pathPrint("Robot Path",self.printer)
           # self.printButton.config(relief='raised')
            self.controlmenu.entryconfig(1, label="Begin Path Recording")
        else:
            #Init the printed text
            self.printer = ""
            self.controlmenu.entryconfig(1, label="Stop Path Recording")
           # self.printButton.config(relief='sunken')

        self.recording = not self.recording
    
    def coords(self,event):
        #Update mouse clicked coords
        self.firstPoint = [event.x,event.y]
    
    def closeWin(self):
        self.win3.destroy()

    def robottoDot(self):
        dx = self.firstPoint[0] - self.primaryRobot.position[0]
        dy = self.firstPoint[1] - self.primaryRobot.position[1]
        self.e5.delete(0,END)
        self.e6.delete(0,END)
        self.e5.insert(0,self.firstPoint[0])
        self.e6.insert(0,self.firstPoint[1])
        self.primaryRobot.abstranslate([dx,dy])
        
    def moveRobot(self):
        try:
            script = self.printed.printMap() #Print out the robot movement values
        except:
            tkMessageBox.showerror("Error!","Robot path is blank!")

        self.setPrimary()
        #Animate the robot according to code
        for x in range(0,len(script)):
            self.primaryRobot.canvas.update()
            if script[x][1] != 0: #Value of movement is not ZERO
                if script[x][0] == 'm': #Translation
                    print "Moving"
                    
                    self.primaryRobot.animtranslate(script[x][1]*self.pixeltoCM[0])

                elif script[x][0] == 'r':
                    print "Rotating"
                    self.primaryRobot.rotate(script[x][1])

            self.primaryRobot.canvas.update()
    
    def __init__(self,parentCanvas,initSize,pixelinit,datuminit,boundsizeinit,robotName,fginit,bginit):
        primaryWindow = initSize
        self.datum = datuminit
        self.boundsize = boundsizeinit
        self.pixeltoCM = pixelinit
        self.recording = False
        self.printer = ""
        self.w = parentCanvas
        self.win3 = Toplevel() #Create a Tkinter window object for controller TOPLEVEL when you have more than one
        self.win3.title("Robot Controller - " + str(robotName)) #set all primary robot parameters
        self.win3.geometry(str(primaryWindow[0])+"x"+str(primaryWindow[1]))
        w3 = Canvas(self.win3,bg="white")
        w3.pack(expand=YES,fill=BOTH)
        #Starting vector
        t5 = Label(w3,text="Starting X: ").grid(row=0,column=0,pady=(20,0))
        self.e5 = Entry(w3) 
        self.e5.insert(0,0)
        self.e5.grid(row=0,column=1,pady=(20,0))
        t6 = Label(w3,text="Starting Y: ").grid(row=1,column=0)
        self.e6 = Entry(w3)
        self.e6.insert(0,0)
        self.e6.grid(row=1,column=1)
        t7 = Label(w3,text="Starting Heading (degrees): ").grid(row=2,column=0)
        self.e7 = Entry(w3)
        self.e7.insert(0,0)
        self.e7.grid(row=2,column=1)
        t8 = Label(w3,text="Gauge (centimeters): ").grid(row=3,column=0)
        self.e8 = Entry(w3)
        self.e8.insert(0,0)
        self.e8.grid(row=3,column=1)
        t9 = Label(w3,text="Length (centimeters): ").grid(row=4,column=0)
        self.e9 = Entry(w3)
        self.e9.insert(0,0)
        self.e9.grid(row=4,column=1)

        Label(w3,text="", bg="white").grid(row=5,column=0) #blank space
        #activate1 = Button(w3, text="Build robot", command=self.setPrimary) #button to set primary robot parameters
        #activate1.grid(row=6,column=0)
        #flip1 = Button(w3, text="Flip sides", command=self.flipPrimary) #Button for it
        #flip1.grid(row=6,column=1) #Assign position
        Label(w3,text="", bg="white").grid(row=7,column=0) #blank space
        setRecord = Button(w3, text="Translate to red dot", command=self.setRecord) #button to draw robot path by clicking co-ordinates
        setRecord.grid(row=8,column=0)
        self.reversegear = IntVar()
        Checkbutton(w3,text="Reverse gear", variable=self.reversegear).grid(row=8,column=1)
        #self.printButton = Button(w3,text="Record Path",command=self.setPath)
        #self.printButton.grid(row=9,column=0) #Must call on separate line!
        
        self.myFont = tkFont.Font(family = 'Helvetica', size = 20, weight = 'bold', slant="italic")
        nametag = Label(w3,text=robotName,bg=bginit,fg=fginit,font=self.myFont)
        nametag.grid(row=9,column=0,columnspan=2,pady=(20,0))

        menubar = Menu(self.win3)
        self.win3.config(menu=menubar)

        robotmenu = Menu(menubar)
        robotmenu.add_command(label="Build Robot", command=self.setPrimary)
        robotmenu.add_command(label="Flip sides", command=self.flipPrimary)
        menubar.add_cascade(label="Robot",menu=robotmenu)
        
        self.controlmenu = Menu(menubar)
        self.controlmenu.add_command(label="Begin Path Recording", command=self.setPath)
        self.controlmenu.add_command(label="Move robot starting position to red dot", command=self.robottoDot)
        self.controlmenu.add_command(label="Move robot according to Robot Path code", command=lambda: thread.start_new_thread(self.moveRobot,()))
        menubar.add_cascade(label="Controls",menu=self.controlmenu)
        
        self.w.bind("<Button 1>",self.coords, add ="+")	#Find co-ordinate
