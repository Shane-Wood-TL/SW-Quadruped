import tkinter as tk
import serial

values_queue = []
s = None #serial port object

#function that handles when the connect button is pressed
def serialConnect():
    global s #pull global serial port
    portName = serialPort.get()
    baudRate = selBaudRate.get()
    baudRate = int(baudRate)
    print(f"Port:{portName} Baud Rate:{baudRate}")

    s = serial.Serial(portName,baudRate,timeout=1)
    

#function that handles when the disconnect button is pressed
def serialDisconnect():
    global s #pull global serial port
    if s and s.is_open:
        s.close()
        print("Disconnected successfully")
    else:
        print("Serial port is not open or not connected")

#function that sends the values over serial
def sendValues(): 
    global s #pull global serial port
    valuetoSend = [ALegHip.get(),ALegKnee.get(),ALegAnkle.get(),
                   BLegHip.get(),BLegKnee.get(),BLegAnkle.get(),
                   CLegHip.get(),CLegKnee.get(),CLegAnkle.get(),
                   DLegHip.get(),DLegKnee.get(),DLegAnkle.get()]
    floatValues = []
    toSendString = ""
    for i in range(len(valuetoSend)):
        floatValues.append(float(valuetoSend[i])) #ensure float, and send
        toSendString += "," + valuetoSend[i]
    toSendString = toSendString[1:]
    print(toSendString)
    values_queue = toSendString

    s.write(values_queue.encode()) #encode the value that is about to be send next in the queue and send it
    s.write(b"\n")  #add a linebreak when a command is done




#window setup
root = tk.Tk()
root.title("Motor Offset Calibration")

#top row text / inputs
serialPortLabel = tk.Label(root, text="SerialPort:")
serialPortLabel.grid(row=0,column=0)
#serial port name entry (such as COM3)
serialPort = tk.Entry(root, width=10)
serialPort.grid(row=0,column=1)


#serial port baud rate entry (such as 9600)
selBaudRate = tk.Entry(root)
selBaudRate.grid(row=0,column=2)

#connect and disconnect buttons
connect = tk.Button(text="Connect",command=serialConnect)
connect.grid(row=0,column=3)
connect = tk.Button(text="Disconnect",command=serialDisconnect)
connect.grid(row=0,column=4)

labelRow = 1

motorNamesColumn = 0
#labels for motor positions
hip = tk.Label(root,text="Hip")
hip.grid(row=2,column=motorNamesColumn)

knee = tk.Label(root,text="Knee")
knee.grid(row=3,column=motorNamesColumn)

ankle = tk.Label(root,text="Ankle")
ankle.grid(row=4,column=motorNamesColumn)

names = ("Aleg","Bleg","Cleg","Dleg")
startingRow = labelRow


#a leg values
aLegRow = 1
ALeg = tk.Label(root,text=names[0])
ALeg.grid(row=startingRow,column=aLegRow)
ALegHip = tk.Entry(root,width=5)
ALegHip.grid(row=startingRow+1, column=aLegRow, padx=5, pady=5)
ALegHip.insert(0, "0") 
ALegKnee = tk.Entry(root,width=5)
ALegKnee.grid(row=startingRow+2, column=aLegRow, padx=5, pady=5)
ALegKnee.insert(0, "0")
ALegAnkle = tk.Entry(root,width=5)
ALegAnkle.grid(row=startingRow+3, column=aLegRow, padx=5, pady=5)
ALegAnkle.insert(0, "0")

#b leg values
BLegRow = 2
BLeg = tk.Label(root,text=names[1])
BLeg.grid(row=startingRow,column=BLegRow)
BLegHip = tk.Entry(root,width=5)
BLegHip.grid(row=startingRow+1, column=BLegRow, padx=5, pady=5)
BLegHip.insert(0, "0")
BLegKnee = tk.Entry(root,width=5)
BLegKnee.grid(row=startingRow+2, column=BLegRow, padx=5, pady=5)
BLegKnee.insert(0, "0")
BLegAnkle = tk.Entry(root,width=5)
BLegAnkle.grid(row=startingRow+3, column=BLegRow, padx=5, pady=5)
BLegAnkle.insert(0, "0")

#c leg values
CLegRow = 3
CLeg = tk.Label(root,text=names[2])
CLeg.grid(row=startingRow,column=CLegRow)
CLegHip = tk.Entry(root,width=5)
CLegHip.grid(row=startingRow+1, column=CLegRow, padx=5, pady=5)
CLegHip.insert(0, "0")
CLegKnee = tk.Entry(root,width=5)
CLegKnee.grid(row=startingRow+2, column=CLegRow, padx=5, pady=5)
CLegKnee.insert(0, "0")
CLegAnkle = tk.Entry(root,width=5)
CLegAnkle.grid(row=startingRow+3, column=CLegRow, padx=5, pady=5)
CLegAnkle.insert(0, "0")

#d leg values
DLegRow = 4
DLeg = tk.Label(root,text=names[3])
DLeg.grid(row=startingRow,column=DLegRow)
DLegHip = tk.Entry(root,width=5)
DLegHip.grid(row=startingRow+1, column=DLegRow, padx=5, pady=5)
DLegHip.insert(0, "0")
DLegKnee = tk.Entry(root,width=5)
DLegKnee.grid(row=startingRow+2, column=DLegRow, padx=5, pady=5)
DLegKnee.insert(0, "0")
DLegAnkle = tk.Entry(root,width=5)
DLegAnkle.grid(row=startingRow+3, column=DLegRow, padx=5, pady=5)
DLegAnkle.insert(0, "0")


#send button
sendValues = tk.Button(text="Send Values",command=sendValues)
sendValues.grid(row=labelRow+5,column=1,columnspan=len(names)+1)

#keep the window open
root.mainloop()



