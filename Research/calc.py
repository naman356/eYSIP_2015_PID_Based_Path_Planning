import math
import serial

ser=serial.Serial(2)

def send_value(value):
    print value
    data1 = value/100
    data2 = value%100/10
    data3 = value%10
    print data1,data2,data3
    
    if data1 == 0:
        ser.write("0")
    elif data1 == 1:
        ser.write("1")
    elif data1 == 2:
        ser.write("2")
    elif data1 == 3:
        ser.write("3")
    elif data1 == 4:
        ser.write("4")
    elif data1 == 5:
        ser.write("5")
    elif data1 == 6:
        ser.write("6")
    elif data1 == 7:
        ser.write("7")
    elif data1 == 8:
        ser.write("8")
    #elif data1 == 9:
    else : 
        ser.write("9")    

    if data2 == 0:
        ser.write("0")
    elif data2 == 1:
        ser.write("1")
    elif data2 == 2:
        ser.write("2")
    elif data2 == 3:
        ser.write("3")
    elif data2 == 4:
        ser.write("4")
    elif data2 == 5:
        ser.write("5")
    elif data2 == 6:
        ser.write("6")
    elif data2 == 7:
        ser.write("7")
    elif data2 == 8:
        ser.write("8")
    #elif data2 == 9:
    else :
        ser.write("9")

    if data3 == 0:
        ser.write("0")
    elif data3 == 1:
        ser.write("1")
    elif data3 == 2:
        ser.write("2")
    elif data3 == 3:
        ser.write("3")
    elif data3 == 4:
        ser.write("4")
    elif data3 == 5:
        ser.write("5")
    elif data3 == 6:
        ser.write("6")
    elif data3 == 7:
        ser.write("7")
    elif data3 == 8:
        ser.write("8")
    #elif data3 == 9:
    else :
        ser.write("9")
k=10

ser.write("7")
while(k):
    k=k-1
    #print k
k=1000
ser.write("8")
while(k):
    k=k-1
    #print k
k=1000
ser.write("9")
while(k):
    k=k-1
    #print k
k=1000
ser.write("0")
while(k):
    k=k-1
    #print k
k=1000
ser.write("2")
while(k):
    k=k-1
    #print k
k=1000
ser.write("9")
while(k):
    k=k-1
    #print k
k=1000
ser.write("0")
while(k):
    k=k-1
    #print k
k=1000
ser.write("F")
while(k):
    k=k-1
    #print k
k=1000
ser.write("9")
while(k):
    k=k-1
    #print k
k=1000
ser.write("8")
while(k):
    k=k-1
    #print k
k=1000
ser.write("9")




'''send_value(65)
ser.write("0")




k=10
send_value(124)
send_value(0)
send_value(35)
'''


