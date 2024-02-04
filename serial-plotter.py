import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def animate(i, dataList, ser):
    ser.write(b'g')                                     # Transmit the char 'g' to receive the Arduino data point
    arduinoData_string = ser.readline().decode('ascii') # Decode receive Arduino data as a formatted string

    try:
        arduinoData_float = float(arduinoData_string)   # Convert to float
        dataList.append(arduinoData_float)              # Add to the list holding the fixed number of points to animate

    except:                                             # Pass if data point is bad                               
        pass

    dataList = dataList[:]                           # Fix the list size so that the animation plot 'window' is x number of points
    
    ax.clear()                                          # Clear last data frame
    ax.plot(range(i, i+len(dataList)), dataList)       # Plot new data frame with dynamic x-axis
    
    ax.set_ylim([-1000, 1000])                          # Set Y axis limit of plot
    ax.set_xlim([i, i + len(dataList)])                # Set x axis limit of plot with dynamic range
    ax.set_title("IMU6050 Data")                       # Set title of figure
    ax.set_ylabel("Degrees/Sec")                       # Set title of y-axis 
    ax.set_xlabel("Time")                               # Set title of x-axis 

dataList = []                                           # Create empty list variable for later use
                                                        
fig = plt.figure()                                      # Create Matplotlib plots fig is the 'higher level' plot window
ax = fig.add_subplot(111)                               # Add subplot to the main fig window

ser = serial.Serial("/dev/ttyUSB1", 115200)            # Establish Serial object with COM port and BAUD rate to match Arduino Port/rate
time.sleep(0.050)                                       # Time delay for Arduino Serial initialization 

# Matplotlib Animation Function that takes care of real-time plot.
# Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
ani = animation.FuncAnimation(fig, animate, frames=100, fargs=(dataList, ser), interval=100) 

plt.show()                                              # Keep Matplotlib plot persistent on the screen until it is closed
ser.close()                                             # Close Serial connection when the plot is closed
