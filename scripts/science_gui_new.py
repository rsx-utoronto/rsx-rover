import serial as ser
import time
import tkinter as tk

BUTTONWIDTH = 20
BUTTONHEIGHT = 2

#send data over serial bus, and print incoming data
def send_data(letter):
    ser.write(letter)
    #change the bit number to 16 if you remove the print statements in the arduino code
    print(ser.read(26)) 


m=tk.Tk()  #main window
m.title('Science Team Drill Controls')  #window title

#connect to arduino
try:
    ser = ser.Serial("COM4", 9600)
    time.sleep(0.5)
except ser.serialutil.SerialException:
    print("Arduino not connected")


#Drill 1 Up
drill_up = tk.Button(m, text='Drill Up', 
                       width=BUTTONWIDTH*2, 
                       height= BUTTONHEIGHT, 
                       bg="yellow",
                       font= ('Helvetica 20 bold'),
                       command= lambda:send_data(b'u'))
#Drill 1 Down
drill_down = tk.Button(m, text='Drill Down', 
                         width=BUTTONWIDTH*2,
                         height= BUTTONHEIGHT, 
                         bg="yellow",
                         font= ('Helvetica 20 bold'),
                         command= lambda:send_data(b'd'))
#Drill 1 FWD
drill_FWD = tk.Button(m, text='Drill CW', #this is usually what we want
                        width=BUTTONWIDTH*2, 
                        height= BUTTONHEIGHT,
                        bg="lightgreen",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'c'))
#Drill 1 REV
drill_REV = tk.Button(m, text='Drill CCW', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="lightgreen",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'ax'))
#   
#attach bit
drill_attachbit = tk.Button(m, text='Attach Bit', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'f'))

#remove bit
drill_removebit = tk.Button(m, text='Remove Bit', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'g'))

# ##rotate changer
bit_changer = tk.Button(m, text='Bit Changer', 
                        width=BUTTONWIDTH*4,
                        height= BUTTONHEIGHT, 
                        bg="pink",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'ow'))

##change delays
delay_100 = tk.Button(m, text='D: 100', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'z100'))
delay_1000 = tk.Button(m, text='D: 1000', 
                        width=BUTTONWIDTH,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'z1000'))
delay_3000 = tk.Button(m, text='D: 3000', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="orange",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'z3000'))

##analysis
testtube = tk.Button(m, text='Test Tube', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b't'))
pump = tk.Button(m, text='Pump', 
                        width=BUTTONWIDTH*2,
                        height= BUTTONHEIGHT, 
                        bg="lightblue",
                        font= ('Helvetica 20 bold'),
                        command= lambda:send_data(b'p'))

# Specify Grid
tk.Grid.columnconfigure(m, index = 0, weight = 1)
tk.Grid.rowconfigure(m, 0,weight = 1)
tk.Grid.columnconfigure(m, index = 1, weight = 1)
tk.Grid.rowconfigure(m, 1,weight = 1)
tk.Grid.columnconfigure(m, index = 2, weight = 1)
tk.Grid.rowconfigure(m, 2,weight = 1)
tk.Grid.columnconfigure(m, index = 3, weight = 1)
tk.Grid.rowconfigure(m, 3,weight = 1)
tk.Grid.rowconfigure(m, 4,weight = 1)
tk.Grid.rowconfigure(m, 5,weight = 1)
tk.Grid.rowconfigure(m, 6,weight = 1)

#place buttons
delay_100.grid(row=0,column=0,columnspan=1,sticky="NSEW")
delay_1000.grid(row=0,column=1,columnspan=1,sticky="NSEW")
delay_3000.grid(row=0,column=2,columnspan=2,sticky="NSEW")

drill_up.grid(row=1,column=0,columnspan=2,sticky="NSEW")
drill_down.grid(row=1,column=2,columnspan=2,sticky="NSEW")
drill_FWD.grid(row=2,column=0,columnspan=2,sticky="NSEW")
drill_REV.grid(row=2,column=2,columnspan=2,sticky="NSEW")

bit_changer.grid(row=3,column=0,columnspan=4,sticky="NSEW")

drill_attachbit.grid(row=4,column=0,columnspan=2,sticky="NSEW")
drill_removebit.grid(row=4,column=2,columnspan=2,sticky="NSEW")

testtube.grid(row=5,column=0,columnspan=2,sticky="NSEW")
pump.grid(row=5,column=2,columnspan=2,sticky="NSEW")

m.mainloop()
