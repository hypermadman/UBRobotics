from Tkinter import *
from ScrolledText import *
import re, tkFileDialog

class pathPrint(object):
    def loadCode(self):
        newFileName = tkFileDialog.askopenfilename(title="Select robot path code...",filetypes=(("Text files","*.txt"),("All Files","*.*")))
        
        if newFileName: #If string is not empty
            #Load text file
            #f = open(newFileName,'r') #Open Read-only
            self.text.delete("1.0",END)
            self.text.insert(INSERT,newFileName.read())
            f.close()

    def saveCode(self):
        savefile = tkFileDialog.asksaveasfile(title="Save robot path code as...",mode='w',defaultextension=".txt") 
        
        if savefile:
            savefile.write = self.text.get("1.0",END)
            savefile.close()

    def __init__(self,title,textinit):
         
        self.wind = Toplevel() #TOPLEVEL when you have more than one
        self.wind.title(title) 
        self.wind.geometry("400x300")
        self.wi = Canvas(self.wind,bg="white")
        self.wi.pack(expand=YES,fill=BOTH)
        self.text = ScrolledText(master=self.wi, height=30, wrap=WORD)
        self.text.insert(INSERT,textinit)
        self.text.pack()
        #self.textinit = textinit #Get the actual text of the path
        
        menubar = Menu(self.wind)
        self.wind.config(menu=menubar)

        robotmenu = Menu(menubar)
        #robotmenu.add_command(label="Load Code", command=self.loadCode)
        #robotmenu.add_command(label="Save Code", command=self.saveCode)
        menubar.add_cascade(label="File",menu=robotmenu)
        

    def printMap(self):
        value = self.text.get("1.0",END) #Return the current value of the scrollable text field

        #Break it apart into single lines
        
        value = value.split('\n')
        #print value

        #Get rid of blank lines

        script = [[0 for x in range(2)] for y in range(len(value))] #init array to contain data
        
        for x in range(0,len(value)-1):
            
            doesMove = value[x].find('move')
            doesRotate = value[x].find('rotate')

            if doesMove != -1:
                #It's moving
                script[x][0] = 'm'
            elif doesRotate != -1:
                #It's rotating
                script[x][0] = 'r'

            #Get the VALUE
            try:
                script[x][1] = int(re.findall(r'[+-]?\d+(?:\.\d+)?',value[x])[0])
            except:
                script.pop(x)

        print script
      
        return script
