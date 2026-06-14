# -*- coding: utf-8 -*-
"""
Created on Thu May 23 12:37:17 2024

@author: SadrSystem
"""


#import matplotlib.pyplot as plt
import turtle
import time

#s = turtle.getscreen()
#path = "LBLSLSRLLRRRSLLSRRLLBLBLSSLBLSBLLBLLR"
#path = "LLLRSRLLSLBLBLLRLBLSBLBLSLSRLLRRRSLLSRRLLBLBLSSLBLSBLLBLLR "
#path ="LBLSLSRLLRRRSLLSRRLLBLBLSSLBLSBLLBLLR"
path = "FFFFFBLLBRFLLRLFRLLLLRLLBLRBLFLLRRLLLFLFLFRBRBFFRLRFF"
print(path[3])
seyed = turtle.Turtle()


sc = turtle.Screen()
sc.reset()
sc.setworldcoordinates(-140,-20,140,200)

turtle.title("Seyed path")

seyed.fillcolor("blue")
seyed.color("black","green")

seyed.left(90)
seyed.forward(20)


for i in path:
    ##time.sleep(1)  
    if i=="L":
      seyed.left(90)
      seyed.forward(20)
    elif i=="S":
       seyed.forward(20)
    elif i=="R":
        seyed.right(90)
        seyed.forward(20)
    elif i=="B":
        seyed.right(180)
        seyed.forward(20)    
      
        
turtle.mainloop() 
turtle.bye()