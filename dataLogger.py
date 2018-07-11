"""
Data Logger
Daniel J. Gonzalez
dgonz@mit.edu
"""

import numpy

class dataLogger():
    myData = []
    running = 0

    def __init__(self, name):
        self.name = name
        self.running = 1
        f = open(name, 'w')
    
    def writeOut(self):
        if(self.running==1):
            self.running = 0
            f = open(self.name, 'a')
            outTxt = []
            for data in self.myData:
                myStr = str(data)
                myStr = myStr.replace('[', '')
                myStr = myStr.replace(']', '')
                #print(myStr) #for debugging
                outTxt.append(myStr+'\n')
                #for e in data:
                #    outTxt.append(str(e) + ' ')
                #outTxt.append('\n')
            f.write(''.join(outTxt))
            f.write('\n')
            self.myData = []
            #numpy.savetxt(name,self.myData)
        else:
            print('Already Done')
            return

    def appendData(self,val):
        self.myData.append(val)
        
    def isRunning(self):
        return self.running

