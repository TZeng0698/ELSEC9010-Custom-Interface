# Author: Taha Tariq Zengin
# Date: 25-08-2025
# Program: ELSEC-9010 Custom Interface
# Version: 1.0
# Description: Interface for fully automated sample irradiation system, ELSEC-9010
import tkinter as tk
from tkinter import PhotoImage,Button, messagebox, filedialog, Canvas, colorchooser
import math
import serial
import time
import threading
from threading import Lock
from pathlib import Path
import sys
from serial.tools import list_ports
import pyautogui
import ast
from PIL import Image, ImageTk

OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH = OUTPUT_PATH / "assets"
RUNS_PATH = OUTPUT_PATH / "runs"


#Default position map
positions = [
    [(4800, 200), (4143, 200), (3486, 200), (2829, 200), (2171, 200), (1514, 200), (857, 200), (200, 200)],
    [(4800, 857), (4143, 857), (3486, 857), (2829, 857), (2171, 857), (1514, 857), (857, 857), (200, 857)],
    [(4800, 1514), (4143, 1514), (3486, 1514), (2829, 1514), (2171, 1514), (1514, 1514), (857, 1514), (200, 1514)],
    [(4800, 2171), (4143, 2171), (3486, 2171), (2829, 2171), (2171, 2171), (1514, 2171), (857, 2171), (200, 2171)],
    [(4800, 2829), (4143, 2829), (3486, 2829), (2829, 2829), (2171, 2829), (1514, 2829), (857, 2829), (200, 2829)],
    [(4800, 3486), (4143, 3486), (3486, 3486), (2829, 3486), (2171, 3486), (1514, 3486), (857, 3486), (200, 3486)],
    [(4800, 4143), (4143, 4143), (3486, 4143), (2829, 4143), (2171, 4143), (1514, 4143), (857, 4143), (200, 4143)],
    [(4800, 4800), (4143, 4800), (3486, 4800), (2829, 4800), (2171, 4800), (1514, 4800), (857, 4800), (200, 4800)]
]
#Default configuration values
withdrawalPosition=(2829, 4800)
acceleration=50
maxSpeed=200
avgHomingTime=50
generalTimeOut=90
pingInterval=10
withdrawalTimeOut=180
homingTimeOut=120
homeAfterSteps=100000

#Scales For UI
guiScale=2
masterColor="#FAFAF9"

simulationQueue=[] # commands waiting to be completed and displayed
instructionQueue = [] # command waiting to be sent
sentQueue=[] # commands sent, waiting for response
unMatchedResponses = [] # responses that did not match any sent command
processedResponses=[] # responses that have been processed in multi-step 
completionTime=[] # timestamps of completed commands
# priorityQueue=[] # not in use yet
movingTimeList=[] # time taken to move to each sample
distanceList=[] # distance to each sample
expectedResponses = {}  # Maps commands to expected sequential responses
commandState = None  # Tracks the current command requiring multi-step confirmation

avgResponseTime = 0.75;  #expected delay in communication (Interface->Microcontroller->Interface)                            
totalTime=0 # total expected time for the whole run
prevIndex=0 # previous sample position index
stepCount=0 # counts the number of steps taken since last home
warningMessageCount=0 # to avoid multiple warning messages
tempSentQueue="" # used as a temporary storage for sentQueue when paused
currentPosition =(None, None)
lastPosition=(None,None)
targetIndex= -1 # target position index
currentSampleIndex= -1
previousSampleIndex= -1
localZigZagIndex=-1 # zigzag index for display
displayZigZag=-1 
mcuID="MCU"

runComplete=False #used in UI
textRetrieved=False #used in progressWindow box texts'
paused=False #triggered by pause button
pauseStatus=False #checks whether the system is in pauseState
pausedPreviously=False # progressWindow UI
thread_running = True  #Controls IOthread
abortStatus=False
manualWindowStatus=False
speedStatus=False
timeOutStatus=False
connectionStatus=True
stopStatus=False
microcontrollerStatus=False     #  only checks whether Microcontroller is responsive, has no direct effect
homingStatus=False                                    
rootStatus=False      #used in UI                                  
configurationWindowStatus=False #used in UI                         
setupWindowStatus=False #used in UI 
progressWindowStatus=False #used in UI 
setupPositionWindowStatus=False #used in UI 
indexMatchFound=False   #to check if returned position matches with positions[]
timeoutHandled = False
progressWindowAbortStatus=False #used in UI
manualButtonsDisabled=False #used in UI
offlineMode=False #no MCU connection at the start

ser=serial.Serial()
IOthread=threading.Thread()
queue_lock = Lock()
#withdrawalStatus=False # not in use yet, but extra precaution should be taken


def relative_to_assets(path: str) -> Path:
    #returns paths to asset files
    return ASSETS_PATH / Path(path)

def relative_to_runs(path: str) -> Path:
    #returns paths to run files
    return RUNS_PATH / Path(path)

def control_thread():
    global thread_running, IOthread,connectionStatus
    if not IOthread.is_alive() and thread_running!=False:
        try:
            IOthread = threading.Thread(target=instruction_queue_handler)
            IOthread.start()
            thread_running=True
            print("Thread revived.")
        except:
            connectionStatus=False
            safe_showerror("Error", "Thread initializiation failed. Please restart the program.")

def stop_thread():
    """Stops IOthread"""
    global thread_running
    thread_running = False # for safe exit
    try:
        if IOthread.is_alive():
            IOthread.join()  
        print("Thread stopped.")
    except RuntimeError as e:
        print(f"Error stopping thread: {e}")

def exit():
    global rootStatus,root
    try:
        if rootStatus:
            rootStatus=False
            root.destroy()
        pseudo_garbage_collector()
        ser.reset_output_buffer() 
        ser.reset_input_buffer()
    except Exception as e:
        print("Error exiting:",e)

def pseudo_garbage_collector():
    """Cleans up global variables"""
    global instructionQueue,simulationQueue,sentQueue,completionTime,currentCommand,expectedResponses,processedResponses,distanceList
    global totalTime,movingTimeList,pauseStatus,runComplete,textRetrieved,microcontrollerStatus,timeOutStatus
    print("clearing cache")
    if simulationQueue or sentQueue or expectedResponses:
        microcontrollerStatus=False
    try:
        instructionQueue.clear()
        simulationQueue.clear()
        sentQueue.clear()
        completionTime.clear()
        expectedResponses.clear()
        processedResponses.clear()
        movingTimeList.clear()
        distanceList.clear()
    except Exception as e:
        print("Failed to clear:",e)
    currentCommand=None
    pauseStatus=False
    runComplete=False
    textRetrieved=False
    timeOutStatus=False
    totalTime=0
    if(connectionStatus==True):
        try:
            ser.reset_output_buffer() 
            ser.reset_input_buffer()
        except:
            print("Error flushing serial buffer")

def find_microcontroller_port():
    global mcuID
    device_descriptions = [
        "microcontroller",
        "microcontroller uno",
        "microcontroller mega",
        "microcontroller nano",
        "microcontroller leonardo",
        "microcontroller microcontroller",
        "ch340",
        "ch341",
        "ft232r",
        "ftdi",
        "cp2102",
        "atmega16u2",
        "acm",
        "tty",
        "nodemcu",
        "esp32",
        "teensy",
        "arduino",
        "raspberry",
        "pi"
    ]
    ports = list_ports.comports()
    time.sleep(1)
    for port in ports:
        desc_lower = port.description.lower()
        print(f"Found port: {port.device} - {port.description}")

        for keyword in device_descriptions:
            if keyword in desc_lower:
                keyword=keyword[0].upper()+keyword[1:]
                mcuID = keyword 
                print(f"Microcontroller found on port: {port.device} (matched: '{mcuID}')")
                return port.device
    return None

def attempt_connecting():
    """Attempt connecting to Microcontroller"""
    global microcontrollerStatus, connectionStatus, homeButton, withdrawalButton, irradiateButton, IOthread,ser
    global timeOutStatus, response_time_out, speedStatus, homingStatus, ser, commandState,stopStatus,offlineMode
    global runButton, instructionQueue, sentQueue, simulationQueue, completionTime, abortStatus,progressWindowAbortStatus

    pseudo_garbage_collector()

    if not connectionStatus:
        port = find_microcontroller_port()
        while port is None and thread_running:
            port = find_microcontroller_port()
            time.sleep(1)
        try:
            ser = serial.Serial(port, 9600, timeout=5)
            time.sleep(1) 
            if not ser.is_open:
                ser.open()
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            connectionStatus = False
            return
        # Reset global states
        timeOutStatus = False
        response_time_out = 0
        speedStatus = False
        homingStatus = False
        commandState = None

        if not abortStatus:
            safe_showinfo("Connection", f"Connected to {mcuID}")
        instructionQueue.clear()
        sentQueue.clear()
        simulationQueue.clear()
        completionTime.clear()

        time.sleep(0.5)  # Wait before starting I/O thread
        connectionStatus = True
        stopStatus=False
        abortStatus = False
        progressWindowAbortStatus = False
        microcontrollerStatus = True 
        offlineMode=True

        try:
            ser.reset_output_buffer() 
            ser.reset_input_buffer()
            startTime=time.time()
            while ser.in_waiting >= 1 and time.time() - startTime < 2:  #extra measure to clear the buffer
                ser.readline()
                time.sleep(0.001)
        except Exception as e:
            print(f"Error flushing serial buffer: {e}")

        return

def connection_lost():
    """Start connection lost process"""
    global microcontrollerStatus, connectionStatus, homeButton, withdrawalButton, irradiateButton
    global runButton, IOthread,ser
    try:
        ser.close()
    except Exception as e:
        print("Failed to close the serial")
        
    if not abortStatus and not offlineMode:
        safe_showerror("Lost Connection", "Lost Connection")

    microcontrollerStatus = False
    connectionStatus = False
    attempt_connecting()
    return

def read_config_file():
    """
    Retrieve configuration settings from the "config.txt" file.
    """

    global acceleration,maxSpeed,generalTimeOut,withdrawalTimeOut,homingTimeOut,homeAfterSteps,withdrawalPosition,pingInterval,guiScale,masterColor
    config_file = OUTPUT_PATH / "config.txt"
    try:
        with open(config_file, "r") as file:
            config = file.readlines()

        acceleration = int(config[0].split("=")[1].strip())
        maxSpeed = int(config[1].split("=")[1].strip())
        generalTimeOut = int(config[2].split("=")[1].strip()) 
        withdrawalTimeOut = int(config[3].split("=")[1].strip()) 
        homingTimeOut = int(config[4].split("=")[1].strip())  
        pingInterval = int(config[5].split("=")[1].strip())
        homeAfterSteps = int(config[6].split("=")[1].strip())
        try:
            # Convert string to tuple
            withdrawalPosition = ast.literal_eval(config[7].split("=")[1].strip())
        except (ValueError, SyntaxError):
            safe_showerror("Error"," Error reading the config.txt\nThe withdrawalPosition is not a valid tuple.")
            return
        guiScale=float(config[8].split("=")[1].strip())
        masterColor=str(config[9].split("=")[1].strip())
    except:
        if(safe_askyesno("Error","Failed to read the config file.\n Would you like to reset your configuration?")):
            #overwrite the globals and create a new config file
            acceleration=50
            maxSpeed=200
            generalTimeOut=90
            withdrawalTimeOut=180
            homingTimeOut=120
            pingInterval=20
            homeAfterSteps=100000
            withdrawalPosition=(2829, 4800)
            guiScale=2
            write_config_file(acceleration, maxSpeed,generalTimeOut,withdrawalTimeOut,homingTimeOut,pingInterval,homeAfterSteps,withdrawalPosition,guiScale,masterColor)
        else:
            return   

def write_config_file(configAcceleration=1, configMaxSpeed=1,configGeneralTimeOut=1,
configWithdrawalTimeOut=1,configHomingTimeOut=1,configPingInterval=1,configHomeAfterSteps=1,configWithdrawalPosition=(1,1),configGuiScale=2,configMasterColor="white"):
    """
    Used for saving the configuration into config.txt
    """

    global speedStatus,acceleration,maxSpeed,generalTimeOut,withdrawalTimeOut,homingTimeOut,homeAfterSteps,withdrawalPosition,pingInterval,guiScale,masterColor
    global root
    config_file = OUTPUT_PATH / "config.txt"
    speedStatus=False

    try:
        with open(config_file, "w") as file:
            if configAcceleration != "":
                file.write(f"acceleration={configAcceleration}\n")
                acceleration=int(configAcceleration)

            if configMaxSpeed != "":
                file.write(f"maxSpeed={configMaxSpeed}\n")
                maxSpeed=int(configMaxSpeed)

            if configGeneralTimeOut != "":
                file.write(f"generalTimeOut={configGeneralTimeOut}\n")
                generalTimeOut=int(configGeneralTimeOut)

            if configWithdrawalTimeOut != "":
                file.write(f"withdrawalTimeOut={configWithdrawalTimeOut}\n")
                withdrawalTimeOut=int(configWithdrawalTimeOut)

            if configHomingTimeOut != "":
                file.write(f"homingTimeOut={configHomingTimeOut}\n")
                homingTimeOut=int(configHomingTimeOut)

            if pingInterval != "":
                file.write(f"pingInterval={configPingInterval}\n")
                pingInterval=int(configPingInterval)

            if configHomeAfterSteps != "":
                file.write(f"homeAfterSteps={configHomeAfterSteps}\n")
                homeAfterSteps=int(configHomeAfterSteps)

            if configWithdrawalPosition != "":                                                          #correct!!!!
                file.write(f"withdrawalPosition={configWithdrawalPosition}\n")
                withdrawalPosition=configWithdrawalPosition
            
            if configGuiScale != "":
                file.write(f"guiScale={configGuiScale}\n")
                guiScale=float(configGuiScale)
                root.tk.call("tk", "scaling", guiScale)
            if configMasterColor != "":
                file.write(f"masterColor={configMasterColor}\n")
                masterColor=configMasterColor
            safe_showinfo("Success", "Your changes have been saved.")
        return
    except Exception as e:
        tk.messagebox("Error", f"Failed to create config file at {config_file}: {e}")


def read_position_file():
    """Retrieve the position map from the position.txt"""
    global positions
    position_file=OUTPUT_PATH / "position.txt"
    try:
        with open(position_file, "r") as file:
            matrix = []
            for line in file:
                # Parse each line into a list of tuples
                row = [
                    tuple(map(int, pos.strip("()").split(",")))
                    for pos in line.split()
                ]
                matrix.append(row)
            positions=matrix
    except FileNotFoundError:
        messagebox.showerror("Error",f"File '{position_file}' not found.\nPlease generate a new positions from configuration window(gear symbol at the corner of main window) and restart.")
    except ValueError:
        messagebox.showerror("Error","Positions are not in the expected format. Ensure positions are in the format (x,y) and restart.")
        return


def write_position_file(tempPositions):
        """Saves the position map to a file."""
        position_file = OUTPUT_PATH / "position.txt"
        try:
            with open(position_file, "w") as file:
                for row in tempPositions:
                    file.write(" ".join(f"({x},{y})" for x, y in row) + "\n")
            read_position_file()
            safe_showinfo("Success", "Positions saved to position.txt!")
        except IOError:
            messagebox.showerror("Error", "Failed to save positions to file!")


def compute_moving_time(pos1, pos2):
    """
    Computes the time required to move a specified distance using given acceleration and max speed.
    Handles the case where the motor doesn't reach max speed if distance is too short.
    """
    global maxSpeed, acceleration, avgResponseTime
    global movingTimeList, distanceList
    #print(f"pos1:{pos1} pos2:{pos2}" )
    # Calculate the total distance 
    # but if the stepper only moves in X or Y, then max of differences is typical)
    total_distance = abs(pos2[0] - pos1[0]) + abs(pos2[1] - pos1[1])
    distanceList.append(total_distance)

    # Move distance is the max of x or y if you're stepping diagonally at the same rate
    move_distance = max(abs(pos2[0] - pos1[0]), abs(pos2[1] - pos1[1]))
    if move_distance == 0:
        # No movement
        movingTimeList.append(0)
        return 0

    # Distance needed to reach maxSpeed at given acceleration
    s_a = (maxSpeed ** 2) / (2 * acceleration)  # distance to accelerate from 0 to maxSpeed

    if move_distance < 2 * s_a:
        # --- Case 1: The stepper DOES NOT reach maxSpeed ---
        # Use symmetrical acceleration and deceleration
        time_total = 2.0 * math.sqrt(move_distance / acceleration)

    else:
        # --- Case 2: The stepper reaches maxSpeed, then cruises, then decelerates ---
        # 1) Accelerate distance = s_a
        # 2) Decelerate distance = s_a
        # 3) Cruise distance = move_distance - 2*s_a
        time_acc = maxSpeed / acceleration
        time_dec = time_acc
        cruise_distance = move_distance - 2*s_a
        time_cruise = cruise_distance / maxSpeed

        time_total = time_acc + time_cruise + time_dec

    # Add overhead or response time
    #print(f"time_total:{time_total} ")
    time_total += avgResponseTime
    movingTimeList.append(time_total)
    return time_total


def order_positions(activationTimes):
    """ Used for filtering and ordering the "Serial" instruction"""
    validIndices = [i for i, time in enumerate(activationTimes) if time > 0]  # Append indices of non-zero activation times
    sampleTimes = []
    processedIndices = []  # To track the traversal order
    movingTimeList.clear()

    #reordering the sample order
    for row in range(8):  
        for col in range(8):
            if row%2==0:  
                idx = row * 8 + col  # Calculate index in the flattened grid
            else:
                idx=row*8+7-col
            if idx in validIndices:
                processedIndices.append(idx)

    tempCurrentPosition=currentPosition
    # Process samples in the new order
    for idx in processedIndices:
        row, col = divmod(idx, 8)  # Map index 
        targetPosition = positions[row][col]  # Get target position (x, y)



        # Calculate the time it takes to move to sample
        moveTime = compute_moving_time(tempCurrentPosition,targetPosition)

        # Total time for this sample
        sampleTime = moveTime + activationTimes[idx]
        sampleTimes.append(sampleTime) # not in use but could be useful

        # Update current position
        tempCurrentPosition = targetPosition
    
    return processedIndices

def match_sample_index(tempPosition):
    """Match the given position with a position from the positions map"""
    global currentPosition,indexMatchFound,irradiateButton
    global lastPosition

    tempFlag=False
    if(simulationQueue):
        if(simulationQueue[0].split()[0]=="I" and lastPosition!=currentPosition):
            safe_showerror("Critical Error","IRRADIATING WRONG SAMPLE! CHECK CURRENT POSITION!")
            #instruction_handler("Abort")  up to you !
    

    for i, row in enumerate(positions):
        for j, position in enumerate(row):
            if position == tempPosition:
                currentSampleIndex = i * 8 + j
                tempFlag=True
                lastPosition=tempPosition
    if(tempFlag):
        indexMatchFound=True
        print(tempPosition,"found")
        return currentSampleIndex
    else:
        print(tempPosition,"position not found")
        indexMatchFound=False 
        return -1                        

def set_target_index(instruction):
    """Get the target index from the instruction.Used in manualWindow for color updates"""
    global targetIndex
    parts = instruction.split()
    if parts[0] == "M":
        if len(parts) >= 3:
            x = int(parts[1])
            y = int(parts[2])
            targetIndex = match_sample_index((x, y))
            print(f"target index:{targetIndex}")
        else:
            print("Invalid M instruction format:", parts)
    elif parts[0] == "I":
        targetIndex = currentSampleIndex

def run_time_check(localRemainingTime):
    """
    Verifies if the expected time for the run is met or within the boundaries of maximumTimeOut
    """
    global runComplete, totalTime
    runComplete=True # since it is done after every successful run
    maximumTimeOut=max(withdrawalTimeOut,max(homingTimeOut,generalTimeOut))
    localRemainingTime=abs(localRemainingTime)
    totalTime=0
    if localRemainingTime>maximumTimeOut:
        print(localRemainingTime, maximumTimeOut)
        safe_showerror("Run Times Do Not Match", "Run Times Do Not Match\nPlease check maxSpeed and acceleration.\nMake sure they are not too high or low.")
        if safe_askyesno("Reset", "Would you like to reset?"):
            pseudo_garbage_collector()
            instruction_handler("Abort")
            #no need to forceHome, abort will already force the user
        else:
            return

def correct_time_entry(timeEntry): #currently in use by instruction_handler("Irradiate"). Moved to prevent overcrowding
    if(type(timeEntry)==str):
        timeEntry =timeEntry.replace(" ","") # Remove extra spaces
    try:
        irradiationTime = float(timeEntry)  # Parse input as float
        if irradiationTime < 0 or irradiationTime > 100000:
            raise ValueError("Enter a valid number between 0.00 and 100000.00")
        # Round to 2 significant figures
        irradiationTime = round(irradiationTime, 2)
    except ValueError:
        safe_showerror("Invalid Input", "Please enter a valid number between 0.00 and 100000.00 .")
        irradiationTime = 0.0
    return irradiationTime

def convert_to_zigzag_index(index=currentSampleIndex):
    """Converts sample index to zigzag index for display"""
    global displayZigZag 
    if (index//8)%2==0:
        displayZigZag=index+1
    else:
        displayZigZag=(index//8 +1)*8- index%8
    return displayZigZag

#Pop-ups do not appear on top sometimes.Below are some correction
def get_safe_parent():
    global manualWindow, manualWindowStatus, root
    try:
        # If manualWindow is active, use it as the parent
        if manualWindowStatus and manualWindow.winfo_exists():
            return manualWindow
        # Otherwise, try to get the currently focused widget's toplevel
        widget = root.focus_get()
        if widget:
            return widget.winfo_toplevel()
    except:
        pass
    return root

def safe_showinfo(title, message):
    parent = get_safe_parent()
    try:
        parent.attributes("-topmost", 1)  # Make parent window topmost
        parent.update()  # Force update of the event loop
        messagebox.showinfo(title, message, parent=parent)
        parent.attributes("-topmost", 0)  # Reset topmost
        parent.focus_force()  # Ensure parent regains focus
    except Exception as e:
        print(f"Error in safe_showinfo: {e}")
        messagebox.showinfo(title, message)  # Fallback to default

def safe_showerror(title, message):
    parent = get_safe_parent()
    try:
        parent.attributes("-topmost", 1)
        parent.update()
        messagebox.showerror(title, message, parent=parent)
        parent.attributes("-topmost", 0)
        parent.focus_force()
    except Exception as e:
        print(f"Error in safe_showerror: {e}")
        messagebox.showerror(title, message)

def safe_askyesno(title, message):
    parent = get_safe_parent()
    try:
        parent.attributes("-topmost", 1)
        parent.update()
        result = messagebox.askyesno(title, message, parent=parent)
        parent.attributes("-topmost", 0)
        parent.focus_force()
        return result
    except Exception as e:
        print(f"Error in safe_askyesno: {e}")
        return messagebox.askyesno(title, message)


def response_queue_handler(response):  #handled by IOthread
    """Matches responses with the current instruction"""
    global commandState, expectedResponses, processedResponses
    if sentQueue:
        currentInstruction = sentQueue[0].split()[0]

    # Define the expected sequences for multi-step commands
    multiStepMap = {
        "FH": ["Next", "Home"],
        "W": ["Next", "Withdrawal"],
        "M": ["Next", "Done"],
        "I": ["Next", "Done"]
    }

    # Handle multi-step commands
    if commandState:
        # Skip already-processed responses
        if response in processedResponses:
            print(f"Redundant response ignored: {response}")
            return False
        
        # Check if the response matches the next expected step
        if expectedResponses:

            if response == expectedResponses.get(commandState, [])[0]:
                processedResponses.append(response)  # Mark this response as processed
                expectedResponses[commandState].pop(0)  # Remove the completed step

                if not expectedResponses[commandState]:  # All steps completed
                    commandState = None
                    processedResponses.clear()  # Reset processed steps
                    if sentQueue:
                        sentQueue.pop(0)
                return True
            
            elif(response.startswith("Pos")): #Ghost Ping
                return True

            else:
                print(f"Unexpected response: {response}. Expected: {expectedResponses.get(commandState, [])}")
                if(abortStatus==False and stopStatus==False):
                    unMatchedResponses.append(response)
                return False

    # Initialize multi-step state if required
    if currentInstruction in multiStepMap:
        if response == multiStepMap[currentInstruction][0]:  # First expected response
            commandState = currentInstruction
            expectedResponses[commandState] = multiStepMap[currentInstruction][1:]  # Set remaining steps
            processedResponses = [response]  # Start tracking processed responses
            return True

    # Handle single-step commands
    responseMap = {
        "Next": ["maxSpeed", "acceleration"],
        "Stop": ["S"],
        "Pong": ["Ping"],
        "Pos": ["P"],
        "Done": ["I", "M"],
        "Withdrawal": ["W"],
        "Home": ["FH"],
    }
    if currentInstruction in responseMap.get(response, []):
        print(f"Single-step command matched: {response}")
        if sentQueue:
            sentQueue.pop()
        return True

    return False

def response_handler(): # handled by IOthread
    """Executes functions depending on the responses and checks for timeout""" 
    global timeOutStatus, microcontrollerStatus, response_time_out, commandState,currentCommand
    global timeoutHandled,currentCommand,totalTime

    if not sentQueue:  # No response is expected
        return
    run_time = time.time()
    lastPingTime=time.time()
    localPingInterval=pingInterval
    numberOfAttempts = 1
    pinged=False
    response_time_out=pingInterval
    while sentQueue and thread_running and not pauseStatus:  # Ensure sentQueue is not empty
        try:
            if ser.in_waiting > 0:
                timeoutHandled = False
                responseHeader, response = read_serial()
                if responseHeader == "Stop":
                    handle_stop()
                    return

                if response_queue_handler(responseHeader): # Match the response with the sent instruction
                    if responseHeader == "Next":
                        response_time_out = update_timeout()

                    if responseHeader == "Home":
                        print("Homing completed")
                        handle_homing()
                        return
                
                    elif responseHeader == "Withdrawal":
                        print("Withdrawal completed")
                        simulationQueue.pop(0)
                        totalTime=0
                        return

                    elif responseHeader == "Done":
                        response_time_out = 5
                        handle_simulation_queue()
                        return
                    
                    elif responseHeader == "Pos": # Used also as a ghost ping
                        handle_Pos(response) #Updates timeout as well
                        lastPingTime=time.time() #New ping time
                        pinged=False

                elif(abortStatus==False and stopStatus==False): # Only take the unmatched responses  when the user did not abort or stop
                    unMatchedResponses.append(str(response))
                    time.sleep(0.01)
            
            #PING while irradiating
            elif(sentQueue):
                if(sentQueue[0].split()[0] == "I"):
                    if(not pinged and lastPingTime + localPingInterval < time.time()):
                        ping()
                        lastPingTime=time.time()
                        pinged=True

                    elif pinged and ((time.time() - lastPingTime) > localPingInterval):
                        numberOfAttempts, localPingInterval = handle_timeout(lastPingTime, numberOfAttempts)

            elif (time.time() - run_time)> response_time_out:
                numberOfAttempts, response_time_out = handle_timeout(run_time, numberOfAttempts)

            elif not ser.is_open and connectionStatus:
                connection_lost()
                return

            else:
                time.sleep(0.2)
        except serial.SerialException:
            print("Serial exception")
            print("LOST CONNECTION 2")
            connection_lost()
            time.sleep(1)
    time.sleep(0.001)

def instruction_queue_handler():# handled by IOthread
    """
    Continuously processes instructions and sends them to the Microcontroller. IOthread
    """
    counter=0
    queue_lock = Lock()
    with queue_lock:
        while thread_running:
            if(not pauseStatus):
                if (ser.is_open==False and connectionStatus):
                    print("LOST CONNECTION 5")
                    connection_lost()
                if not sentQueue:
                    if instructionQueue:#instructionQueue is not empty and sentQueue is empty
                        currentInstruction = instructionQueue[0]
                        if(ser.is_open):
                            sentQueue.append(instructionQueue.pop(0))
                            try:
                                ser.write((f"{currentInstruction}\n").encode("utf-8"))
                            except serial.SerialException:
                                print("LOST CONNECTION 6")
                                connection_lost()
                                return
                            print(f"Sent: {currentInstruction}")
                            set_target_index(currentInstruction)

                    elif (not instructionQueue and rootStatus and (counter>pingInterval/10)):  #idling timing based on tick rate+1s
                        instruction_handler("getPosition") 
                        time.sleep(2)
                        counter=0       
                    elif(counter%5==0):
                        counter=counter+1        
                    else:
                        response_handler()
                        time.sleep(1)
                        counter=counter+1
                else: #sentQueue is not empty
                    response_handler()
            else:
                time.sleep(1)#thread put to pause

def instruction_handler(instructionType, instruction=(-1, -1), activationTimes=-1):#both threads may enter!
    """
    Used for appending instructions
    """
    global remainingTime,timeOutStatus,microcontrollerStatus,response_time_out,homingStatus,speedStatus
    global simulationQueue,currentSampleIndex,homeAfterSteps,currentPosition,ser,connectionStatus,abortStatus
    global stopStatus,commandState,IOthread,indexMatchFound,totalTime,thread_running,commandState,sentQueue,pauseStatus
    global tempSentQueue,expectedResponses,progressWindow
    if ser.is_open==False and connectionStatus:
        print("LOST CONNECTION 4")
    
    elif connectionStatus==False:
        return
    else:
        
        if instructionType == "getPosition":
            instructionQueue.append("P")
            return
        
        #Low cost instructions
        microcontrollerStatus=False



        if instructionType == "Clear":  #### NOT IN USE
            instructionQueue.append("C")
            print("Clear queued")
            return
        
        elif instructionType == "Stop" and not stopStatus  and not abortStatus:
            if("FH" in instructionQueue or "FH" in sentQueue):
                homingStatus=False
            stopStatus=True
            indexMatchFound=False
            print("stopping")
            if not paused:
                pseudo_garbage_collector()
            sentQueue.clear()
            expectedResponses.clear()
            commandState=None
            instructionQueue.insert(0,"S")
            time.sleep(0.1)
            return
        
        elif instructionType == "Pause" and not abortStatus and not stopStatus and not pauseStatus:
            if(sentQueue):
                if(sentQueue[0].split()[0]=="I"):
                    tempSentQueue = f"I {int(activationTimes * 100) / 100:.2f}"
                else:
                    tempSentQueue=sentQueue[0]
            thread_running=False # to stop the thread just temporarily
            sentQueue.clear()
            expectedResponses.clear()
            commandState=None
            instruction_handler("Stop")
            print("Recovering command:",tempSentQueue)
            if tempSentQueue[0].split()[0]=="I":
                if float(tempSentQueue.split()[1])<=0:
                    tempSentQueue="I 0"
            if instructionQueue[0].split()[0]=="S" and tempSentQueue != None:
                instructionQueue.insert(1,tempSentQueue)
            elif tempSentQueue != None:
                #instruction_handler("forceHoming") #to ensure the position is known
                instructionQueue.insert(0,tempSentQueue)
            else:
                print("No command to recover")
            thread_running=True

            def _on_stop_check():
                global pauseStatus
                if stopStatus:
                    progressWindow.after(500, _on_stop_check)
                else:
                    print("Pause successful!")
            progressWindow.after(10, _on_stop_check)
            return
        

        #Mid cost instructions
        if instructionType == "Abort" and not abortStatus:
            indexMatchFound=False
            abortStatus=True
            time.sleep(0.1)
            ser.close()
            time.sleep(0.05)
            if configurationWindowStatus==True:
                instructionQueueTK.delete(0, tk.END)
                for response in instructionQueue:
                    instructionQueueTK.insert(tk.END,str(response))
            instruction_handler("getPosition")
            return

        elif instructionType == "forceHoming" :
            indexMatchFound = False
            simulationQueue.insert(0,"FH")
            instructionQueue.insert(0,"FH")
            print("Homing queued")
            currentPosition=(1,1)
            totalTime+=avgHomingTime
            homingStatus=True
            return
        
        if speedStatus==False and not stopStatus and not abortStatus:
            instructionQueue.append(f"maxSpeed {maxSpeed}")
            instructionQueue.append(f"acceleration {acceleration}")
            speedStatus=True

        # Home if aborted recently or if homing has never been done
        if homingStatus==False and not stopStatus and not abortStatus:
            sentQueue.clear()
            if isinstance(activationTimes, list) and all(time == 0 for time in activationTimes):
                print("No irradiation. Skipping homing...")
            else:
                instruction_handler("forceHoming")
            currentPosition=(1,1)# Initial position is assumed to be not known in this case. It is a small correction for better estimation
            #in moving time

        if instructionType == "Withdrawal":
            print("Withdrawal queued")
            indexMatchFound = False
            simulationQueue.append("W") 
            instructionQueue.append(f"W {withdrawalPosition[0]} {withdrawalPosition[1]}")
            return 

        elif instructionType == "Irradiate":
            if isinstance(activationTimes, float) or isinstance(activationTimes, str):
                activationTimes=correct_time_entry(activationTimes)
                formattedActivationTimes = activationTimes  # Format to two decimal places
            else:
                formattedActivationTimes = str(activationTimes)  # Convert integer to string
            simulationQueue.append(f"I {formattedActivationTimes}")
            instructionQueue.append(f"I {formattedActivationTimes}")

            totalTime += activationTimes
            return
        
        elif instructionType == "Manual":
            indexMatchFound = False
            manualInstructions = f"M {instruction[0]} {instruction[1]}"                     
            simulationQueue.append(manualInstructions)
            instructionQueue.append(manualInstructions)
            calculatedTime=compute_moving_time(currentPosition,instruction)                                              
            totalTime+=calculatedTime                                       
            return
            
        elif instructionType == "Serial": #Multicommand instructions
            processedIndices=order_positions(activationTimes)
            instruction=processedIndices
            for idx in instruction:  # indices received as instruction
                row, col = divmod(idx, 8)  
                x, y = positions[row][col]  
                subInstruction = f"{x} {y}"
                command = "M " + subInstruction
                simulationQueue.append(command)
                irradiationTime = activationTimes[idx]
                instructionQueue.append(command)
                instruction_handler("Irradiate", activationTimes=irradiationTime)
            totalTime+=sum(movingTimeList)         
            return processedIndices         

def read_serial():
    global ser
    try:
        response = ser.readline().decode('utf-8').strip()
    except serial.SerialException:
        connection_lost()
        print("LOST CONNECTION 3")
        return "", ""
    try:
        responseHeader = response.split()[0]
    except:
        print("Bad response")
        responseHeader = "Null"
    return responseHeader,response

def handle_timeout(start_time, numberOfAttempts):
    global timeoutHandled, sentQueue, instructionQueue, microcontrollerStatus, timeOutStatus, root

    response_time_out = generalTimeOut
    if timeoutHandled:
        return numberOfAttempts, response_time_out  # Exit if timeout already handled
    timeoutHandled = True  # Mark the timeout as handled
    elapsed_time = time.time() - start_time  # Calculate elapsed time

    def show_timeout_askyesno():
        result = safe_askyesno(
            "Microcontroller did not respond in time.",
            f"Keep waiting? Time elapsed: {int(elapsed_time)} seconds."
        )
        return result

    def show_timeout_error():
        safe_showerror(
            "TimeOut Error",
            f"TimeOut after waiting {int(elapsed_time)} seconds."
        )

    result = show_timeout_askyesno()
    if result:
        response_time_out += min(60 * numberOfAttempts, 300)  # Cap at 5 min
        timeoutHandled = False
        numberOfAttempts += 1
        return numberOfAttempts, response_time_out
    else:
        timeOutStatus = True
        print(f"TimeOut after waiting {int(elapsed_time)} seconds.")
        pseudo_garbage_collector()
        microcontrollerStatus = False
        # Schedule the error message in the main thread
        root.after(0, show_timeout_error)
        return numberOfAttempts, response_time_out

def update_timeout():
    """Updates the timeout for the current command"""
    if sentQueue:
        if sentQueue[0].split()[0] == "I":
            return float(sentQueue[0].split()[1]) + generalTimeOut
        elif sentQueue[0].split()[0] == "M":
            if movingTimeList:
                return movingTimeList[0]+generalTimeOut
            else:
                return generalTimeOut
        elif sentQueue[0].split()[0] == "W":
            return withdrawalTimeOut+generalTimeOut
        elif sentQueue[0].split()[0] == "FH":
            return homingTimeOut+generalTimeOut
    else:
        return pingInterval    

def ping():
    """sends getPosition command without appending to queue(ghost command). Used as a ping"""
    try:
        ser.write("P\n".encode())
        print("Ping sent")
    except:
        connection_lost()
    return time.time()

def handle_simulation_queue():
    """Pops the the matching command from the simulationQueue"""
    global simulationQueue,completionTime,stepCount,distanceList,homingStatus,targetIndex,currentPosition,currentSampleIndex
    try:
        if simulationQueue:
            if simulationQueue[0].split()[0] == "I":
                completionTime.append(time.localtime()) #only if it is irradiation
            
            elif simulationQueue[0].split()[0] == "M":
                x=int(simulationQueue[0].split()[1])
                y=int(simulationQueue[0].split()[2])
                currentPosition=(x,y)
                currentSampleIndex=match_sample_index((x,y))
                if movingTimeList:
                    movingTimeList.pop(0)
                if distanceList:
                    stepCount+=distanceList[0]
                    distanceList.pop(0)
                    if stepCount==homeAfterSteps:
                        homingStatus=False
            simulationQueue.pop(0)
        if not simulationQueue:  # Run is complete
            print("Sim Queue is empty")
            return
    except Exception as e:
        print("Pop from handle_sim_que failed",e)

def handle_stop():
    global response_time_out, stepCount, stopStatus,sentQueue,pauseStatus

    stopStatus=False
    if not paused:
        pseudo_garbage_collector()
        instruction_handler("getPosition")

    else:
        pauseStatus=True
        if sentQueue:#remove the last instruction manually
            sentQueue.pop()

    print("Stopped")
        
def handle_homing():
    global response_time_out, stepCount, homingStatus
    response_time_out = 5
    stepCount = 0
    homingStatus=True
    handle_simulation_queue()


def handle_Pos(response):
    global microcontrollerStatus,timeOutStatus,currentSampleIndex,currentPosition
    try:
        x = int(response.split()[1])
        y = int(response.split()[2])
    except Exception as e:
        print("Error getting position",e)
        return
    microcontrollerStatus = True
    timeOutStatus = False
    currentPosition=(x,y)
    currentSampleIndex=match_sample_index((x,y))

def update_ui(dots=""):
    global runButton,instructionQueueTK,unMatchedResponsesTK,unMatchedResponses,manualAbortButton,manualAbortButton,manualStopButton
    global sampleButtons,irradiateButton,previousSampleIndex,manualWindowProcessLabel,progressWindowProcessLabel
    global displayZigZag,sentQueueTK,irradiationTimeTK,progressWindowPauseButton,warningMessageCount,rootConnectionLabel
    global manualWindowConnectionLabel, setupWindowConnectionLabel

    if rootStatus:
        rootConnectionLabel.config( bg="green" if microcontrollerStatus else masterColor,
    text=f"{mcuID} - Not Connected" if not connectionStatus 
    else f"{mcuID} - Ready" if microcontrollerStatus 
    else f"{mcuID} - Busy")
        
    if configurationWindowStatus:#update Configuration Window
        currentPositionTK.config(text=f"Last Position:\n{currentPosition}\n")
        instructionQueueTK.delete(0, tk.END)
        sentQueueTK.config(text=f"Sent Queue:      \n{sentQueue}      \n")
        for response in instructionQueue:
            instructionQueueTK.insert(tk.END, str(response))
        unMatchedResponsesTK.delete(0, tk.END)
        if(len(unMatchedResponses)>5 and warningMessageCount<1):
            warningMessageCount=1
            safe_showerror("Warning",f"Too many unmatched response!\nRestart Microcontroller")
        for response in unMatchedResponses:
            unMatchedResponsesTK.insert(tk.END, str(response))

    if(simulationQueue):# If there is an ongoing process update the label accordingly
        if(simulationQueue[0].split()[0] == "I"):
            processType=f"Irradiating sample {displayZigZag}{dots}"

        elif(simulationQueue[0].split()[0] == "M"):
            processType=f"Moving to sample {displayZigZag}{dots}"

        elif(simulationQueue[0].split()[0] == "W"):
            processType=f"Moving to Withdrawal Position{dots}"
        
        elif(simulationQueue[0].split()[0] == "FH"):
            processType=f"Homing{dots}"

        elif(stopStatus or abortStatus):
            processType="Stopping"
        dots+="."
        if dots == "....":
            dots=""
    elif(connectionStatus==False):
        processType=f"No connection"
    else:
        processType=""
    if(runComplete and not timeOutStatus):
        processType="Run Complete!"
    if timeOutStatus:
        processType="Run failed"
    if setupWindowStatus:# Disable run button if the microcontroller is not ready

        setupWindowConnectionLabel.config(
        bg="green" if microcontrollerStatus else masterColor,
        text=f"{mcuID} - Not Connected" if not connectionStatus 
        else f"{mcuID} - Ready" if microcontrollerStatus 
        else f"{mcuID} - Busy")

        if connectionStatus and microcontrollerStatus and not simulationQueue:# no need to add stopStatus/abortStatus: microcontrollerStatus=stopStatus
            runButton.config(state="normal")
        else:
            runButton.config(state="disabled")
    if progressWindowStatus:#update Progress Window
        progressWindowProcessLabel.config(text=processType)
        if not stopStatus and not abortStatus:
            progressWindowPauseButton.config(state="normal")#DISABLED TEMPORARILY
        else:
            progressWindowPauseButton.config(state="disabled")
        if progressWindowAbortStatus:
            if not connectionStatus and not abortStatus:
                progressWindowAbortButton.config(state="normal")
            else:
                progressWindowAbortButton.config(state="disabled")
    if manualWindowStatus:#update Manual Window
        manualWindowProcessLabel.config(text=processType)
        manualWindowConnectionLabel.config(bg="green" if microcontrollerStatus else masterColor,
        text=f"{mcuID} - Not Connected" if not connectionStatus 
        else f"{mcuID} - Ready" if microcontrollerStatus 
        else f"{mcuID} - Busy")
        if not stopStatus and connectionStatus and not abortStatus and not simulationQueue:
            manualWindow_enable_all()#enable all buttons
        else:
            manualWindow_disable_all()#disable all buttons
            if(stopStatus and not abortStatus):
                manualStopButton.config(state="disabled")
            elif(abortStatus):
                manualStopButton.config(state="disabled")
                manualAbortButton.config(state="disabled")          
            else:
                manualStopButton.config(state="normal")
    # Update the current sample button
        if(sentQueue):
            if targetIndex >=0:#extract target index
                target_canvas, target_circle_id, target_text_id= sampleButtons[targetIndex]
                if(sentQueue[0].split()[0] == "I"):# if the last sent instruction is "Irradiating"
                    target_canvas.itemconfig(target_circle_id, fill="red")  # Color the irradiated sample button in red
                elif(sentQueue[0].split()[0] == "M"):
                    target_canvas.itemconfig(target_circle_id, fill="orange")
        if(currentSampleIndex>=0 and len(sentQueue)==0):
            current_canvas, current_circle_id, current_text_id = sampleButtons[currentSampleIndex]
            current_canvas.itemconfig(current_circle_id, fill="green")

        # Reset the previous buttons' color
            if previousSampleIndex!=currentSampleIndex: 
                if previousSampleIndex>=-1: 
                    previous_canvas, previous_circle_id, previous_text_id = sampleButtons[previousSampleIndex]
                    previous_canvas.itemconfig(previous_circle_id, fill="gray")  # Reset circle to default
                previousSampleIndex = currentSampleIndex

        # enable irradiateButton if there is a valid position and no other instruction on line
        if(indexMatchFound and len(sentQueue)==0 and not simulationQueue):
            displayZigZag = convert_to_zigzag_index(currentSampleIndex)
            irradiateButton.config(text=f"Irradiate Sample {displayZigZag}",state="normal")

    control_thread() # check IOthread, do not delete!
    if thread_running:
        root.after(750,lambda:update_ui(dots)) #update rate


def open_setup_window():
    global runButton, setupWindowStatus, press_timer, setupWindowConnectionLabel, setupWindow
    try:
        press_timer = None
        last_focused_entry = None
        root.withdraw()
        def go_back():
            global setupWindowStatus, press_timer, last_focused_entry
            setupWindowStatus = False
            setupWindow.destroy()
            root.deiconify()

        def run_setup():
            """Gathers sample times from the setup window and transitions to the progress window."""
            pseudo_garbage_collector()
            try:
                activationTimes = [0]*64
                for i in range(8):
                    for j in range(8):
                        idx = int(indexEntries[i][j].get()) if indexEntries[i][j].get() else 0 #set invalid entries to 0
                        idx -= 1
                        if 0 <= idx < len(timeEntries)*len(timeEntries[0]):
                            val = timeEntries[idx//4][idx%4].get()
                            wait_time = float(val) if val.strip() else 0
                            activationTimes[i*8 + j] = wait_time
                open_progress_window(activationTimes, setupWindow)
            except ValueError:
                pass

        def save_setup():
            try:
                file_path = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text Files","*.txt")])
                if file_path:
                    with open(file_path,"w") as f:
                        f.write("Time Table(s)\n")
                        for row in timeEntries:
                            f.write(" ".join(e.get() for e in row)+"\n")
                        f.write("Sample Times(index)\n")
                        for row in indexEntries:
                            f.write(" ".join(e.get() for e in row)+"\n")
                    safe_showinfo("Save Successful","Configuration saved successfully.")
            except Exception as e:
                messagebox.showerror("Error",f"An error occurred while saving: {e}")

        def load_setup():
            try:
                file_path = filedialog.askopenfilename(filetypes=[("Text Files","*.txt")])
                if file_path:
                    with open(file_path,"r") as f:
                        lines = f.readlines()
                        t1 = lines.index("Time Table(s)\n")+1
                        t2 = lines.index("Sample Times(index)\n")+1
                        for i,line in enumerate(lines[1:t1+2]):
                            vals = line.split()
                            for j,v in enumerate(vals):
                                timeEntries[i][j].delete(0,tk.END)
                                timeEntries[i][j].insert(0,v)
                        for i,line in enumerate(lines[t2:t2+8]):
                            vals = line.split()
                            for j,v in enumerate(vals):
                                indexEntries[i][j].delete(0,tk.END)
                                indexEntries[i][j].insert(0,v)
            except Exception as e:
                messagebox.showerror("Error",f"An error occurred while loading: {e}")

        def validate_index(value):
            if value == "":
                return True  
            try:
                value = float(value.strip())
                
                if value.is_integer() and 0 <= value <= 8:
                    return True  
            except ValueError:
                pass  
            return False

        def validate_row_apply_index(value, widget_name):
            if value == "":
                return True  # allow deletion
            if value.isdigit():
                num = int(value)
                if 1 <= num <= 8:
                    try:
                        widget = root.nametowidget(widget_name)
                        row_index = rowApplyEntries.index(widget)
                        for entry in indexEntries[row_index]:
                            entry.delete(0, tk.END)
                            entry.insert(0, value)
                    except Exception as e:
                        print("Validation error:", e)
                    return True
            return False

        def validate_time(value):
            if value == "":
                return True
            try:
                float_value = float(value)
                if 0 <= float_value <= 100000:
                    return True
                return False
            except ValueError:
                return False 
        def clean_entry(widget_name):
            """
            Strips '.0' from the value and ensures it's an integer.
            Called after validation via <FocusOut>.
            """
            try:
                # Retrieve the widget using its name
                widget = setupWindow.nametowidget(widget_name)
                value = float(widget.get().strip())  # Convert to float for processing

                # Convert to integer and update the entry
                widget.delete(0, tk.END)
                widget.insert(0, str(int(value)))
            except ValueError:
                pass  # Ignore invalid values
        def delete_current_entry(event=None):#needed for delete button shortcut(-)
            widget = setupWindow.focus_get()
            if isinstance(widget, tk.Entry):
                widget.delete(0, tk.END)

        def set_last_focused_entry(entry):
            nonlocal last_focused_entry
            last_focused_entry = entry

        def get_focused_position():
            w = root.focus_get()
            for rr in range(len(allEntries)):
                for cc in range(len(allEntries[rr])):
                    if allEntries[rr][cc] == w:
                        return rr, cc
            return None, None

        def move_focus(rstep, cstep):
            rr, cc = get_focused_position()
            if rr is None or cc is None:
                return

            # Calculate new row and column indices
            nr, nc = rr + rstep, cc + cstep

            # Handle wrapping when moving horizontally
            if nc >= len(allEntries[rr]):
                # Move to the first column of the next row
                nc = 0
                nr += 1
            elif nc < 0:
                # Move to the last column of the previous row
                nr -= 1
                if nr >= 0:
                    nc = len(allEntries[nr]) - 1

            # Handle vertical movement (up/down)
            if rstep != 0:
                if nr < 0:
                    nr = 0
                elif nr >= len(allEntries):
                    nr = len(allEntries) - 1

                # Adjust the column index for the new row
                if nc >= len(allEntries[nr]):
                    nc = len(allEntries[nr]) - 1

            # Ensure indices are valid
            nr = max(0, min(len(allEntries) - 1, nr))  # Stay within row bounds
            nc = max(0, min(len(allEntries[nr]) - 1, nc))  # Stay within column bounds

            # Set focus to the new position
            entry = allEntries[nr][nc]
            entry.focus_set()
            entry.select_range(0, tk.END)  # Select all text
            entry.icursor(tk.END)          # Move cursor to end

        def release_button():
            """
            Stops the adaptive increment process when the button is released.
            """
            global press_timer
            if press_timer:
                setupWindow.after_cancel(press_timer)
                press_timer = None

        setupWindowBG=masterColor
        setupWindow = tk.Toplevel(root, bg=setupWindowBG)
        setupWindow.resizable(False, False)
        icon = PhotoImage(file=relative_to_assets("image_2.png"))
        setupWindow.iconphoto(False, icon)
        setupWindow.title("Setup")
        setupWindow.protocol("WM_DELETE_WINDOW", go_back)
        windowWidth = int(700 + 920 * (guiScale - 1))
        windowHeight = int(600+ 400*(guiScale-1))
        setupWindowOffset=f"+{int((screenWidth / 2) - (windowWidth / 2))}+{int((screenHeight / 2) - (windowHeight / 2)-25)}"
        setupWindowGeometry=f"{windowWidth}x{windowHeight}{setupWindowOffset}"
        setupWindow.geometry(setupWindowGeometry)
        setupWindow.grid_rowconfigure(0, weight=0)
        setupWindow.grid_rowconfigure(1, weight=1)
        setupWindow.grid_rowconfigure(2, weight=0)
        setupWindow.grid_rowconfigure(3, weight=1)
        setupWindow.grid_rowconfigure(4, weight=1)
        setupWindow.grid_rowconfigure(5, weight=1)

        IndexLabelFrame = tk.Frame(setupWindow, bg=setupWindowBG)
        IndexLabelFrame.grid(row=0, column=0, padx=int(1*guiScale), pady=int(0.5*guiScale))

        indexFrame = tk.Frame(setupWindow, bg=setupWindowBG)
        indexFrame.grid(row=1, column=0, padx=int(2*guiScale), pady=int(2*guiScale))

        timeLabelFrame = tk.Frame(setupWindow, bg=setupWindowBG)
        timeLabelFrame.grid(row=2, column=0, padx=int(1*guiScale), pady=int(0.5*guiScale))

        timeFrame = tk.Frame(setupWindow, bg=setupWindowBG)
        timeFrame.grid(row=3, column=0, padx=int(2*guiScale), pady=int(2*guiScale))

        buttonFrame = tk.Frame(setupWindow, bg=setupWindowBG)
        buttonFrame.grid(row=4, column=0, padx=1, pady=1)

        buttonFrame.grid_columnconfigure(0,weight=1)
        buttonFrame.grid_columnconfigure(1,weight=1)
        buttonFrame.grid_columnconfigure(2,weight=4)
        buttonFrame.grid_columnconfigure(3,weight=1)
        buttonFrame.grid_columnconfigure(4,weight=1)

        incrementButtonFrame = tk.Frame(buttonFrame, bg=setupWindowBG)
        incrementButtonFrame.grid(row=0, column=0, padx=5, pady=5)  

        premidbuttonFrame = tk.Frame(buttonFrame, bg=setupWindowBG)
        premidbuttonFrame.grid(row=0, column=1, padx=2, pady=5)

        irradiationButtonFrame = tk.Frame(buttonFrame, bg=setupWindowBG)
        irradiationButtonFrame.grid(row=0, column=3, padx=5, pady=5 )

        aftermidbuttonFrame = tk.Frame(buttonFrame, bg=setupWindowBG)
        aftermidbuttonFrame.grid(row=0, column=4, padx=35, pady=5)

        controlButtonFrame = tk.Frame(buttonFrame, bg=setupWindowBG)
        controlButtonFrame.grid(row=0, column=5, padx=5, pady=2)  


        footerFrame = tk.Frame(setupWindow, bg=setupWindowBG)
        footerFrame.grid(row=5, column=0, padx=5, pady=2,sticky="ew")

        # Add empty spacers on left and right
        footerFrame.grid_columnconfigure(0, weight=0)  
        footerFrame.grid_columnconfigure(1, weight=6)  
        footerFrame.grid_columnconfigure(2, weight=1) 

        validateRowApplyIndex = setupWindow.register(validate_row_apply_index)
        validateIndex = setupWindow.register(validate_index)
        validateTime = setupWindow.register(validate_time)
        #Upper frame
        tk.Label(IndexLabelFrame, font=("Times New Roman",16,"bold"), text="Time Table (0.00-100000.00s)", 
                    bg=setupWindowBG).grid(row=1, column=4, columnspan=8, pady=5)

        #Time entries 1 to 8
        timeEntries = []
        for i in range(2):
            row = []
            for j in range(4):
                pairFrameTime = tk.Frame(indexFrame, bg=setupWindowBG)
                pairFrameTime.grid(row=i + 2, column=j, padx=int(5*guiScale), pady=int(0.5*guiScale), sticky="w")

                tk.Label(pairFrameTime, font=("Times New Roman", 16), bg=setupWindowBG,
                        text=str(4 * i + j + 1)).pack(side="left", padx=1,pady=1)

                e = tk.Entry(pairFrameTime, font=("Times New Roman", 16), width=int(4*guiScale), validate="key",borderwidth=2,justify="center",
                            validatecommand=(validateTime, "%P"))
                e.pack(side="right")
                e.insert(0, "0")
                e.bind("<FocusIn>", lambda event, entry=e: set_last_focused_entry(entry))

                row.append(e)
            timeEntries.append(row)

        #Lower frame
        tk.Label(timeLabelFrame, font=("Times New Roman",16,"bold"), text="Sample Time (1-8)", bg=setupWindowBG).grid(row=6, column=4, columnspan=8, pady=10)

        #Index entries
        indexEntries = []
        rowApplyEntries = []
        c = 1

        for i in range(8):
            if i % 2 == 1:
                labels = list(range(c + 7, c - 1, -1))
            else:
                labels = list(range(c, c + 8))

            row = []

            for j in range(8):
                pairFrameIndex = tk.Frame(timeFrame, bg=setupWindowBG)
                pairFrameIndex.grid(row=i + 7, column=j, padx=int(10*guiScale), pady=int(1*guiScale), sticky="w")

                label_text = str(labels[j]) + ("  " if labels[j] <= 9 else "")
                tk.Label(pairFrameIndex, font=("Times New Roman", 16), bg=setupWindowBG, text=label_text).pack(side="left")

                e = tk.Entry(pairFrameIndex, font=("Times New Roman", 16), width=int(3*guiScale),borderwidth=2, validate="key",justify="center",
                            validatecommand=(validateIndex, "%P"))
                e.pack(side="right", padx=(5, 0))
                e.insert(0, "0")
                e.bind("<FocusIn>", lambda event, entry=e: set_last_focused_entry(entry))
                e.bind("<FocusOut>", lambda event, entry=e: clean_entry(entry))

                row.append(e)

            c += 8

            # Row apply entry
            a = tk.Entry(timeFrame, font=("Times New Roman", 16,"bold"), width=5, validate="key",borderwidth=2,justify="center",
                        validatecommand=(validateRowApplyIndex, "%P", "%W"))
            a.grid(row=i + 7, column=8, padx=10, pady=5, sticky="w")

            rowApplyEntries.append(a)
            indexEntries.append(row)

        #need to place all widgets into one list in an order such that we can navigate around with arrow keys
        allEntries=[]
        for row in timeEntries:
            allEntries.append(row)
        for idx, row in enumerate(indexEntries):
            allEntries.append(row + [rowApplyEntries[idx]])

        def increment_buttons():
            keyboardButton.destroy()

            def update_entry_value(delta):
                """
                Updates the value of the currently focused entry by the given delta.
                Applies constraints to ensure valid input.
                """
                try:
                    # Get the currently focused widget
                    entry = root.focus_get()

                    # Ensure the focused widget is an Entry
                    if not isinstance(entry, tk.Entry):
                        raise AttributeError("Focused widget is not an Entry")
                except AttributeError as e:
                    print(f"Exception occurred: {e}")
                    # Set focus to the default entry (0, 0) in allEntries
                    entry = allEntries[0][0]
                    entry.focus_set()

                if entry:
                    try:
                        # Get the current value from the focused entry
                        value = float(entry.get().strip()) if entry.get().strip() else 0

                        # Apply the delta and constrain the value to valid bounds
                        new_value = max(0, min(100000, value + delta))  # Example range: [0, 100000]

                        # Update the entry with the new value
                        entry.delete(0, tk.END)
                        entry.insert(0, str(new_value))
                    except ValueError:
                        pass  # Ignore invalid input


            def press_and_hold(action, initial_delay=200, repeat_delay=50, acceleration=50):
                """
                Executes the action immediately and repeats it with decreasing delay as the button is held.
                """
                def repeat():
                    nonlocal repeat_delay
                    action()  # Perform the action
                    repeat_delay = max(50, repeat_delay - acceleration)  # Speed up over time
                    global press_timer
                    press_timer = setupWindow.after(repeat_delay, repeat)

                # Execute the action immediately
                action()

                # Start adaptive repeating after the initial delay
                global press_timer
                repeat_delay = initial_delay
                press_timer = setupWindow.after(initial_delay, repeat)


            def repeat_increment(f):
                global press_timer
                f()
                press_timer = setupWindow.after(100, lambda: repeat_increment(f))

                    
            b1 = tk.Button(incrementButtonFrame, text="+0.1", font=("Times New Roman",12), width=6)
            b1.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_entry_value(0.1)))
            b1.bind("<ButtonRelease-1>", lambda e: release_button())
            b1.grid(row=0, column=0, padx=2, pady=2)

            b2 = tk.Button(incrementButtonFrame, text="+1", font=("Times New Roman",12), width=6)
            b2.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_entry_value(1)))
            b2.bind("<ButtonRelease-1>", lambda e: release_button())
            b2.grid(row=0, column=1, padx=2, pady=2)

            b3 = tk.Button(incrementButtonFrame, text="+10", font=("Times New Roman",12), width=6)
            b3.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_entry_value(10)))
            b3.bind("<ButtonRelease-1>", lambda e: release_button())
            b3.grid(row=0, column=2, padx=2, pady=2)

            b4 = tk.Button(incrementButtonFrame, text="+100", font=("Times New Roman",12), width=6)
            b4.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_entry_value(100)))
            b4.bind("<ButtonRelease-1>", lambda e: release_button())
            b4.grid(row=1, column=0, padx=2, pady=2)

            b5 = tk.Button(incrementButtonFrame, text="+1000", font=("Times New Roman",12), width=6)
            b5.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_entry_value(1000)))
            b5.bind("<ButtonRelease-1>", lambda e: release_button())
            b5.grid(row=1, column=1, padx=2, pady=2)

            b6 = tk.Button(incrementButtonFrame, text="+10000", font=("Times New Roman",12), width=6)
            b6.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_entry_value(10000)))
            b6.bind("<ButtonRelease-1>", lambda e: release_button())
            b6.grid(row=1, column=2, padx=2, pady=2)

            b7 = tk.Button(incrementButtonFrame, text="-0.1", font=("Times New Roman",12), width=6)
            b7.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_entry_value(-0.1)))
            b7.bind("<ButtonRelease-1>", lambda e: release_button())
            b7.grid(row=2, column=0, padx=2, pady=2)

            b8 = tk.Button(incrementButtonFrame, text="-1", font=("Times New Roman",12), width=6)
            b8.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_entry_value(-1)))
            b8.bind("<ButtonRelease-1>", lambda e: release_button())
            b8.grid(row=2, column=1, padx=2, pady=2)

            b9 = tk.Button(incrementButtonFrame, text="-10", font=("Times New Roman",12), width=6)
            b9.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_entry_value(-10)))
            b9.bind("<ButtonRelease-1>", lambda e: release_button())
            b9.grid(row=2, column=2, padx=2, pady=2)

            b10 = tk.Button(incrementButtonFrame, text="-100", font=("Times New Roman",12), width=6)
            b10.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_entry_value(-100)))
            b10.bind("<ButtonRelease-1>", lambda e: release_button())
            b10.grid(row=3, column=0, padx=2, pady=2)

            b11 = tk.Button(incrementButtonFrame, text="-1000", font=("Times New Roman",12), width=6)
            b11.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_entry_value(-1000)))
            b11.bind("<ButtonRelease-1>", lambda e: release_button())
            b11.grid(row=3, column=1, padx=2, pady=2)

            b12 = tk.Button(incrementButtonFrame, text="-10000", font=("Times New Roman",12), width=6)
            b12.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_entry_value(-10000)))
            b12.bind("<ButtonRelease-1>", lambda e: release_button())
            b12.grid(row=3, column=2, padx=2, pady=2)

        keyboardButton = tk.Button(incrementButtonFrame, command=increment_buttons,  # Replace with increment buttons on click
            text="⌨",font=("Times New Roman", 14),bg=setupWindowBG,borderwidth=0,
        )
        keyboardButton.grid(row=0, column=0, columnspan=1, padx=1, pady=1)
        tk.Button(controlButtonFrame, text="↑", font=("Times New Roman",12,"bold"), width=4, 
                    height=2, command=lambda: move_focus(-1,0)).grid(row=0, column=6, padx=5, pady=5)

        tk.Button(controlButtonFrame, text="←", font=("Times New Roman",12,"bold"), width=4, 
                    height=2, command=lambda: move_focus(0,-1)).grid(row=1, column=5, padx=5, pady=5)

        tk.Button(controlButtonFrame, text="→", font=("Times New Roman",12,"bold"), width=4, 
                    height=2, command=lambda: move_focus(0,1)).grid(row=1, column=7, padx=5, pady=5)

        tk.Button(controlButtonFrame, text="↓", font=("Times New Roman",12,"bold"), width=4, 
                    height=2, command=lambda: move_focus(1,0)).grid(row=2, column=6, padx=5, pady=5)

        tk.Label(premidbuttonFrame, text="            ", bg=setupWindowBG).grid(row=0, column=0, padx=5, pady=5)# fillers
        tk.Label(aftermidbuttonFrame, text="          ",bg=setupWindowBG).grid(row=0, column=0, padx=5, pady=5) 

        setupWindowConnectionLabel = tk.Label(
            footerFrame,
            bg="green"if microcontrollerStatus else masterColor,
            font=("Times New Roman", 11,"bold"),
            fg="black",
            width=17,justify="center",
            text=f"{mcuID} - Not Connected" if not connectionStatus 
        else f"{mcuID} - Ready" if microcontrollerStatus 
        else f"{mcuID} - Busy"
            )
        setupWindowConnectionLabel.grid(row=0, column=2, sticky="e", padx=5)

        runButton = tk.Button(irradiationButtonFrame, font=("Times New Roman",14), text="Run", command=run_setup,state="disabled", width=4)
        runButton.grid(row=0, column=3, columnspan=2, padx=5, pady=5)

        saveButton = tk.Button(irradiationButtonFrame, font=("Times New Roman",14), text="Save", command=save_setup, width=4)
        saveButton.grid(row=1, column=2, padx=10, pady=1)

        loadButton = tk.Button(irradiationButtonFrame, font=("Times New Roman",14), text="Load", command=load_setup, width=4)
        loadButton.grid(row=1, column=5, padx=10, pady=1)

        goBackButton = tk.Button(footerFrame, font=("Times New Roman",14), text="Back", command=go_back, width=4)
        goBackButton.grid(row=0, column=0, padx=5, pady=5, sticky="e")

        setupWindow.bind("<ButtonRelease-1>", lambda _: release_button())
        setupWindow.bind("<Up>", lambda event: move_focus(-1, 0))
        setupWindow.bind("<Down>", lambda event: move_focus(1, 0))
        setupWindow.bind("<Left>", lambda event: move_focus(0, -1))
        setupWindow.bind("<Right>", lambda event: move_focus(0, 1))
        setupWindow.bind("-", delete_current_entry) # a new keybind for deleting inputs

        if timeEntries:
            timeEntries[0][0].focus_set()

        if not connectionStatus and microcontrollerStatus:
            runButton.config(state="disabled")
        setupWindowStatus=True
    except Exception as e:
        safe_showerror("Error",f"An error occured opening Setup Window:\n{e}")
        go_back()

def open_progress_window(activationTimes, setupWindow):
    """Opens the progress window and initiates the run process."""
    global timeOutStatus, progressWindowAbortButton, runButton, progressWindowAbortStatus,progressWindowStatus
    global progressWindowProcessLabel,progressWindowPauseButton,progressWindowAbortButton,progressWindow
    
    runButton.config(state="disabled")
    setupWindow.deiconify()

    def stop_process():
        global progressWindowAbortStatus,progressWindowStatus,paused,pauseStatus
        progressWindowStatus = False
        progressWindowAbortStatus = False
        paused=False
        pauseStatus=False
        progressWindow.destroy()
        if simulationQueue:
            instruction_handler("Stop")
        else:
            pseudo_garbage_collector()
        setupWindow.deiconify()

    def progress_decrement_timer( startTime=0,sampleStartTime=0, processedIdx=-1, que=0, dots="",localRemainingTime=0, activationTimes=[0]):
        """Initiates the timer and Updates the progress window every second."""
        global progressWindowAbortStatus,textRetrieved,current_text,totalTime,pausedPreviously
        
        # Edge cases
        if timeOutStatus:#cancel counting if timeOut
            pseudo_garbage_collector()
            return
        
        elif progressWindowAbortStatus:
            pseudo_garbage_collector()
            progressWindowProcessLabel.config(text="Stopping")
            progressWindowRemainingTimeTk.config(text="")
            safe_showinfo("Abort", "Run canceled")
            return
        
        elif not connectionStatus:
            progressWindowProcessLabel.config(text="Connection Lost!")
            return
        
        
        elif paused and sampleStartTime==0 and startTime==0:# Just idle
            progressWindow.after(1000, progress_decrement_timer,  startTime,sampleStartTime, processedIdx, que, dots,localRemainingTime, activationTimes)
            return
        
        elif paused: # Reset the timers and initiate the pause command
            if sampleStartTime==0:
                sampleStartTime=time.time()
            elapsedSampleTime=time.time()-sampleStartTime
            remainingSampleTime=activationTimes[processedIdx]-elapsedSampleTime
            activationTimes[processedIdx]=remainingSampleTime # update activation time for the sample
            if localRemainingTime!=0:
                totalTime=localRemainingTime# new total time
            startTime=0;sampleStartTime=0; 
            instruction_handler("Pause",activationTimes=remainingSampleTime)
            progressWindow.after(250, progress_decrement_timer,  startTime, sampleStartTime, processedIdx, que, dots,localRemainingTime, activationTimes)
            return
        

        #Normal states(moving,irradiating,...)
        dots += "." #for visuals
        if dots == "....":
            dots = ""

        if startTime != 0:
            currentTime = time.time()
            elapsedTime = currentTime - startTime
            localRemainingTime = totalTime - elapsedTime 
            progressWindowRemainingTimeTk.config(text=f"Remaining Time: {math.ceil(localRemainingTime)}s")

        # Update process labels and button states. It may be a good idea to seperate this part
        if (simulationQueue and sentQueue and processedIndices):
            #bypass if paused previously irradiate does not match because recovered irradiate command is I {remainingSampleTime} whereas old: I {activationTime}
            if (simulationQueue[0] == sentQueue[0]) or pausedPreviously:
                try:
                    processedIdx = processedIndices[que]
                    convert_to_zigzag_index(processedIdx)
                    row, col = divmod(processedIdx, 8)#to be displayed
                except Exception as e:
                    print("An exception ocurred:",e)

                if not textRetrieved: # without it, the text will overflow
                    current_text = sampleDisplay[row][col].cget("text")
                    textRetrieved=True

                if simulationQueue[0].split()[0] == "I":
                    if startTime == 0:
                        startTime = time.time()
                    if sampleStartTime==0:
                        sampleStartTime=time.time()

                    elapsedSampleTime=time.time()-sampleStartTime
                    remainingSampleTime=int(activationTimes[processedIdx]-elapsedSampleTime)

                    sampleDisplay[row][col].config(bg="red", fg="black",
                    text=f"{current_text}\nR:{remainingSampleTime}s")
                    
                if simulationQueue[0].split()[0]=="M": 
                    if startTime == 0:#initatie the main timer
                        startTime = time.time()
                    sampleStartTime=0 # reset the sample timer
                    sampleDisplay[row][col].config(bg="orange", fg="black")

                if que < len(completionTime) and que < len(processedIndices):
                    try:
                        sampleDisplay[row][col].config(
                            bg="green", fg="white",
                            text=f"{current_text}\n{time.strftime('%H:%M:%S', completionTime[que])}"
                        )
                        sampleStartTime=0
                        textRetrieved=False
                        pausedPreviously=False
                        que += 1
                    except Exception as e:
                        print(f"Error updating sample display: {e}")
                        

                # Schedule the next update
                progressWindow.after(500, progress_decrement_timer,  startTime,sampleStartTime, processedIdx, que, dots,localRemainingTime, activationTimes)
                return
            
            else:
                # Waiting for initialization
                progressWindowRemainingTimeTk.config(text=f"Initializing{dots}")
                progressWindow.after(250, progress_decrement_timer, startTime, sampleStartTime, processedIdx, que, dots,localRemainingTime, activationTimes)
                return
        
        #Run complete   
        elif not simulationQueue and not progressWindowAbortStatus:
            if processedIdx and processedIndices:
                if (len(completionTime) != 0):
                    while (que == (len(completionTime) - 1) or len(completionTime)>que):
                        processedIdx = processedIndices[que]
                        row, col = divmod(processedIdx, 8)
                        
                        sampleDisplay[row][col].config(
                            bg="green", fg="white",
                            text=f"{current_text}\n{time.strftime('%H:%M:%S', completionTime[que])}"
                        )
                        que += 1
            progressWindowAbortButton.config(state="disabled")
            print("Run Complete!")
            run_time_check(localRemainingTime)
            safe_showinfo("Success", "Run Complete!")
            runButton.config(state="normal")
            return

        else:
            progressWindow.after(100, progress_decrement_timer, startTime,sampleStartTime, processedIdx, que, dots,localRemainingTime, activationTimes)
            return
    def save_window_as_image():# Save window as an image
        RUNS_PATH.mkdir(exist_ok=True)
        progressWindow.update()
        x = progressWindow.winfo_rootx()
        y = progressWindow.winfo_rooty()
        width = progressWindow.winfo_width()
        height = progressWindow.winfo_height()
        screenshot = pyautogui.screenshot(region=(x, y, width, height))
        now = time.localtime()
        save_path = relative_to_runs(f"Run {time.strftime('%Y-%m-%d %H-%M-%S', now)}.png")
        screenshot.save(save_path)
        safe_showinfo("Window Saved", f"Window saved to\n {save_path}")

    def abort_run():
        global  progressWindowAbortButton,progressWindowAbortStatus
        progressWindowAbortStatus = True
        progressWindowAbortButton.config(state="disabled")
        instruction_handler("Abort")

    def trigger_pause():#works only when decrement_window is running
        
        global paused,pauseStatus,pausedPreviously
        if paused:
            progressWindowPauseButton.config(text="Pause",bg="red",fg="white",state="disabled")
            pauseStatus=False
            paused=False
            print("Resuming")
            print(sentQueue,instructionQueue)
        else:
            progressWindowPauseButton.config(text="Resume",bg="green",fg="white",state="normal")#DISABLED TEMPORARILY
            paused=True
            pausedPreviously=True
            print("Pausing")
        #Save current run
    
    progressWindow = tk.Toplevel(setupWindow, bg=masterColor)
    progressWindow.title("Progress Window")
    progressWindow.resizable(False, False)
    icon = PhotoImage(file=relative_to_assets("image_2.png"))
    windowWidth = int(640 + 505 * (guiScale - 1))
    windowHeight = int(600+ 350*(guiScale-1))
    progressWindowWindowOffset=f"+{int((screenWidth / 2) - (windowWidth / 2))}+{int((screenHeight / 2) - (windowHeight / 2)-50)}"
    progressWindowGeometry=f"{windowWidth}x{windowHeight}{progressWindowWindowOffset}"
    progressWindow.geometry(progressWindowGeometry)
    progressWindow.iconphoto(True, icon)
    progressWindow.protocol("WM_DELETE_WINDOW", stop_process)

    progressWindowSampleFrame=tk.Frame(progressWindow, bg=masterColor)
    progressWindowSampleFrame.grid(row=0, column=0, padx=5, pady=5,sticky="ew")

    progressWindowProcessFrame=tk.Frame(progressWindow, bg=masterColor)
    progressWindowProcessFrame.grid(row=1, column=0, padx=5, pady=5,sticky="ew")
    progressWindowProcessFrame.grid_columnconfigure(0,weight=0)
    progressWindowProcessFrame.grid_columnconfigure(1,weight=1)
    progressWindowProcessFrame.grid_columnconfigure(2,weight=0)

    progressWindowButtonFrame=tk.Frame(progressWindow, bg=masterColor)
    progressWindowButtonFrame.grid(row=2, column=0, padx=5, pady=5,sticky="ew")
    progressWindowButtonFrame.grid_columnconfigure(0,weight=0)
    progressWindowButtonFrame.grid_columnconfigure(1,weight=2)
    progressWindowButtonFrame.grid_columnconfigure(2,weight=1)
    progressWindowButtonFrame.grid_columnconfigure(3,weight=2)
    progressWindowButtonFrame.grid_columnconfigure(4,weight=0)

    progressWindowRemainingTimeTk = tk.Label(progressWindowProcessFrame, text="Initializing", font=("Times New Roman", 14, "bold"),bg=masterColor)
    progressWindowRemainingTimeTk.grid(row=0, column=1,padx=10, pady=int(1*guiScale),sticky="n")

    progressWindowProcessLabel = tk.Label(progressWindowProcessFrame, text="", font=("Times New Roman", 14, "bold"),bg=masterColor)
    progressWindowProcessLabel.grid(row=1, column=1,padx=10, pady=int(1*guiScale),sticky="n")

    progressWindowPauseButton = tk.Button(progressWindowButtonFrame, text="Pause", width=8,justify="center",
        command=trigger_pause, bg="red", fg="white", font=("Times New Roman", 12),state="disabled")
    progressWindowPauseButton.grid(row=0, column=1, pady=5,padx=5,sticky="e")

    takePictureButton = tk.Button(progressWindowButtonFrame, text="Save Window", width=12,justify="center",
        command=save_window_as_image, bg="green", fg="white", font=("Times New Roman", 12))
    takePictureButton.grid(row=0, column=2, pady=5,padx=5)

    progressWindowAbortButton = tk.Button(progressWindowButtonFrame, text="Abort", width=8,justify="center",
        command=abort_run, bg="red", fg="white", font=("Times New Roman", 12))
    progressWindowAbortButton.grid(row=0, column=3, pady=5,padx=5,sticky="w")
    
    processedIndices=instruction_handler("Serial", activationTimes=activationTimes) #initiate the run

    #Square samples boxes in the progress window yellow bg if being irradiated and white if not
    sampleDisplay = []
    for i in range(8):
        row_widgets = []
        for j in range(8):
            idx = i * 8 + j
            localZigZagIndex=convert_to_zigzag_index(i * 8 + j)
            activation_time = ""
            if idx in processedIndices:
                activation_time = f"{activationTimes[idx]:.2f}s"   
            lbl = tk.Label(
                progressWindowSampleFrame,
                font=("Times New Roman", 12, "bold"),
                text=f"Sample {localZigZagIndex}\n{activation_time}",
                width=9,
                height=3,
                bg="lightyellow" if activation_time else "white",
                relief="solid"
            )
            lbl.grid(row=i, column=j, padx=5, pady=5)
            row_widgets.append(lbl)
        sampleDisplay.append(row_widgets)

    setupWindow.withdraw()
    runButton.config(state="disabled")
    progressWindowAbortStatus = False
    progressWindowStatus = True
    if processedIndices:
        convert_to_zigzag_index(processedIndices[0])
    progress_decrement_timer(activationTimes=activationTimes)
    progressWindow.mainloop()


def open_manual_window(): # Control panel for single step tasks
    """
    Manual Control Window
    """
    global irradiateButton,withdrawalButton,homeButton,sampleButtons,manualWindowStatus,manualRemainingTimeTK,manualAbortButton,manualStopButton
    global manualWindowProcessLabel,manualButtonsDisabled,irradiationTimeTK,keyboardButton,placeholder_active,manualWindowConnectionLabel,manualWindow
    try:
        if not instructionQueue:
            instruction_handler("getPosition")
        root.withdraw()
        def goBack():
            global manualWindowStatus
            try:
                
                manualWindowStatus=False
                sampleButtons.clear()
                if simulationQueue:
                    pseudo_garbage_collector()
                    instruction_handler("Stop")
                else:
                    pseudo_garbage_collector()
                manualWindow.destroy()
                root.deiconify()
            except:
                safe_showerror("Error", "An error occurred while closing the window.")
                if simulationQueue or instructionQueue:
                    instruction_handler("Stop")

        def check_time(value):
            try:
                float_value = float(value) 
                if 0 <= float_value <= 100000:
                    return True
                else:
                    safe_showerror("Error", "Enter a valid number between 0.00 and 100000.00")
                    return False
            except ValueError:
                safe_showerror("Error", "Enter a valid number between 0.00 and 100000.00")
                return False


        def clear_placeholder(event):
            global placeholder_active
            if placeholder_active:
                irradiationTimeTK.delete(0, tk.END)
                placeholder_active = False

        def restore_placeholder(event):
            global placeholder_active
            if irradiationTimeTK.get() == "":
                irradiationTimeTK.insert(0, placeholder_text)
                placeholder_active = True

        manualWindowBG=masterColor
        manualWindow = tk.Toplevel(root, bg=manualWindowBG)
        manualWindow.resizable(False, False)

        manual_window_width = int(guiScale * (1600 / 1.8))
        manual_window_height = int(guiScale * (800 / 1.8))
        x = int((screenWidth / 2) - (manual_window_width / 2))
        y = int((screenHeight / 2) - (manual_window_height / 2)-50)
        geometry = f"{manual_window_width}x{manual_window_height}+{x}+{y-10}"
        manualWindow.geometry(geometry)

        manualWindow.title("Manual Control Window")
        manualWindow.protocol("WM_DELETE_WINDOW", goBack)
        icon = PhotoImage(file=relative_to_assets("image_2.png"))
        manualWindow.iconphoto(False, icon)
        sampleButtons = []

        sampleFrameBG=masterColor
        sampleFrame = tk.Frame(manualWindow, bg=sampleFrameBG)
        sampleFrame.grid(row=0, column=0, padx=5, pady=1)

        buttonFrame = tk.Frame(manualWindow, bg=manualWindowBG)
        buttonFrame.grid(row=0, column=1, padx=5, pady=1)

        footerFrame = tk.Frame(manualWindow,bg=manualWindowBG)
        footerFrame.grid(row=2, column=0, padx=5,columnspan=2, pady=1,sticky="ew")

        progressFrame=tk.Frame(buttonFrame,bg=manualWindowBG)
        progressFrame.grid(row=0, column=0, padx=5, pady=1)

        irradiationButtonFrame = tk.Frame(buttonFrame, bg=manualWindowBG)
        irradiationButtonFrame.grid(row=1, column=0, padx=5, pady=5, sticky="n")  

        incrementButtonFrame = tk.Frame(buttonFrame, bg=manualWindowBG)
        incrementButtonFrame.grid(row=2, column=0, padx=10, pady=10, sticky="n")  

        controlButtonFrame = tk.Frame(buttonFrame, bg=manualWindowBG)
        controlButtonFrame.grid(row=3, column=0, padx=5, pady=2, sticky="n")

        manualWindow.grid_rowconfigure(0, weight=1)  # Sample frame
        manualWindow.grid_columnconfigure(1,weight=1)
        manualWindow.grid_rowconfigure(1, weight=0)  # Control frame
        manualWindow.grid_rowconfigure(2, weight=0)
        manualWindow.grid_rowconfigure(3, weight=0)  # Progress frame
        manualWindow.grid_rowconfigure(4, weight=0)  # Footer frame

        controlButtonFrame.grid_columnconfigure(0, weight=5)  
        controlButtonFrame.grid_columnconfigure(1, weight=0)  
        controlButtonFrame.grid_columnconfigure(2, weight=1)  
        controlButtonFrame.grid_columnconfigure(3, weight=5)  

        irradiationButtonFrame.grid_columnconfigure(0, weight=6) 
        irradiationButtonFrame.grid_columnconfigure(1, weight=1)  
        irradiationButtonFrame.grid_columnconfigure(2, weight=6)

        footerFrame.grid_columnconfigure(0, weight=0) 
        footerFrame.grid_columnconfigure(1, weight=1)  
        footerFrame.grid_columnconfigure(2, weight=0)  




        label_counter = 1
        button_radius = 25*guiScale*.85  # Radius of the circular buttons
        xPadding = 5*guiScale*.85  # Space between buttons
        yPadding = 4*guiScale*.85
        canvas_padding = 2*guiScale*.5  # Additional padding for canvas

        for i in range(8):  # 8 rows for samples
            if i % 2 == 0:  # First row in normal order
                labels = list(range(label_counter, label_counter + 8))
            else:  # Reverse order for subsequent rows
                labels = list(range(label_counter + 7, label_counter - 1, -1))
            for j, label in enumerate(labels):
                labelIndex = 8 * i + j

                # Increase the canvas size slightly to prevent cropping
                canvas = tk.Canvas(
                    sampleFrame,
                    width=(button_radius * 2) + canvas_padding,
                    height=(button_radius * 2) + canvas_padding,
                    bg=sampleFrameBG,
                    highlightthickness=0,
                )

                # Center the circle within the padded canvas
                x, y = (button_radius + canvas_padding // 2), (button_radius + canvas_padding // 2)
                circle = canvas.create_oval(
                    x - button_radius,
                    y - button_radius,
                    x + button_radius,
                    y + button_radius,
                    fill="gray",
                )
                text = canvas.create_text(x, y, text=f"{label}", font=("Times New Roman", 20,"bold"), fill="black")
                canvas.grid(row=i, column=j, padx=xPadding, pady=yPadding)


                # Append canvas to sampleButtons for later reference
                sampleButtons.append((canvas, circle, text))
            label_counter += 8

        manualWindowProcessLabel = tk.Label(progressFrame, font=("Times New Roman", 15,"bold"), text="", bg=manualWindowBG,width=25)
        manualWindowProcessLabel.grid(row=0, column=0, padx=5, pady=1)

        manualRemainingTimeTK = tk.Label(progressFrame, font=("Times New Roman", 15,"bold"), text="", bg=manualWindowBG,width=25)
        manualRemainingTimeTK.grid(row=1, column=0, padx=5, pady=1)

        irradiateButton = tk.Button(
            irradiationButtonFrame,text=f"Irradiate Sample {displayZigZag}" if indexMatchFound and currentSampleIndex>=0 else f"Irradiate Sample -", 
            font=("Times New Roman", 14, "bold"),
            fg="black", width=20, 
            state="normal" if indexMatchFound and currentSampleIndex>=0 else "disabled", 
            command=lambda: (
                manual_run_command(currentSampleIndex, "Irradiate Sample") and irradiateButton.config(state="disabled")
                if check_time(irradiationTimeTK.get()) else None
            ),
        )
        irradiateButton.grid(row=0, column=1, padx=1, pady=1, sticky="ew")
        irradiationTimeTK = tk.Entry(irradiationButtonFrame,font=("Times New Roman", 14, "bold"),fg="black",width=22,borderwidth=2,justify="center")
        irradiationTimeTK.grid(row=1, column=1, padx=1, pady=1, sticky="ew")
        placeholder_text = "Irradiation Duration (s)"
        placeholder_active = True
        irradiationTimeTK.insert(0, placeholder_text)
        irradiationTimeTK.bind("<FocusIn>", clear_placeholder)
        irradiationTimeTK.bind("<FocusOut>", restore_placeholder)
        irradiationTimeTK.bind("<Key>", clear_placeholder) 
        
        withdrawalButton = tk.Button(
            controlButtonFrame, text="Withdraw", font=("Times New Roman", 14), width=10,height=1, state="disabled",
            command=lambda: (
                manual_run_command(processType="Withdrawal")
            )
        )
        withdrawalButton.grid(row=0, column=1, columnspan=2, padx=20, pady=5, sticky="e")  

        
        homeButton = tk.Button(
            controlButtonFrame, text="Home", font=("Times New Roman", 14), width=10,height=1, state="disabled",
            command=lambda: (
                manual_run_command(processType="forceHoming")
            )
        )
        homeButton.grid(row=1, column=1, columnspan=2, padx=20, pady=5, sticky="e")

        
        manualStopButton = tk.Button(
            controlButtonFrame, text="Stop", font=("Times New Roman", 14), width=10,height=1,  
            command=lambda: instruction_handler("Stop"),state="disabled"
        )
        manualStopButton.grid(row=2, column=1, columnspan=2, padx=20, pady=5, sticky="e")

        
        manualAbortButton = tk.Button(
            controlButtonFrame, text="Abort", font=("Times New Roman", 14), bg="red", fg="white", width=10,height=1,  
            command=lambda: instruction_handler("Abort")
        )
        manualAbortButton.grid(row=3, column=1, columnspan=2, padx=20, pady=5, sticky="e")

        
        backButton = tk.Button(
            footerFrame, font=("Times New Roman", 14), text="Back", command=goBack, width=4, 
        )
        backButton.grid(row=0, column=0, padx=5, pady=1,sticky="w")

        manualWindowConnectionLabel=tk.Label(footerFrame, bg="green" if microcontrollerStatus else masterColor,
        text=f"{mcuID} - Not Connected" if not connectionStatus 
        else f"{mcuID} - Ready" if microcontrollerStatus 
        else f"{mcuID} - Busy",
        font=("Times New Roman",13,"bold"),justify="center", 
        width=18)
        manualWindowConnectionLabel.grid(row=0,column=2,padx=5,pady=5,sticky="e")
        
        def increment_buttons():
            def update_irradiation_time(delta):
                """
                Updates the irradiationTimeTK entry with the given delta.
                """
                try:
                    # Get the current value from the entry
                    value = float(irradiationTimeTK.get().strip()) if irradiationTimeTK.get().strip() else 0.0

                    # Apply delta and constrain the value if needed
                    new_value = max(0, min(100000, value + delta))  # Prevent negative values

                    # Update the entry with the new value
                    irradiationTimeTK.delete(0, tk.END)
                    irradiationTimeTK.insert(0, f"{new_value:.2f}")  # Format as float with 2 decimals
                except ValueError:
                    # If invalid input, reset the entry to 0
                    irradiationTimeTK.delete(0, tk.END)
                    irradiationTimeTK.insert(0, "0.00")

            def press_and_hold(action, initial_delay=250, repeat_delay=50, acceleration=50):
                """
                Executes the action immediately and repeats it with decreasing delay when the button is held.
                """
                def repeat():
                    nonlocal repeat_delay
                    action()  # Perform the action
                    repeat_delay = max(5, repeat_delay - acceleration)  # Decrease delay to speed up
                    global press_timer
                    press_timer = irradiationTimeTK.after(repeat_delay, repeat)

                # Execute the action immediately
                action()

                # Start adaptive repeating after the initial delay
                global press_timer
                repeat_delay = initial_delay
                press_timer = irradiationTimeTK.after(initial_delay, repeat)

            def release_button():
                """
                Stops the adaptive increment process when the button is released.
                """
                global press_timer
                if press_timer:
                    irradiationTimeTK.after_cancel(press_timer)
                    press_timer = None

            keyboardButton.destroy()
            b1 = tk.Button(incrementButtonFrame, text="+0.1", font=("Times New Roman",12), width=6)
            b1.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_irradiation_time(0.1)))
            b1.bind("<ButtonRelease-1>", lambda e: release_button())
            b1.grid(row=0, column=0, padx=2, pady=2)

            b2 = tk.Button(incrementButtonFrame, text="+1", font=("Times New Roman",12), width=6)
            b2.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_irradiation_time(1)))
            b2.bind("<ButtonRelease-1>", lambda e: release_button())
            b2.grid(row=0, column=1, padx=2, pady=2)

            b3 = tk.Button(incrementButtonFrame, text="+10", font=("Times New Roman",12), width=6)
            b3.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_irradiation_time(10)))
            b3.bind("<ButtonRelease-1>", lambda e: release_button())
            b3.grid(row=0, column=2, padx=2, pady=2)

            b4 = tk.Button(incrementButtonFrame, text="+100", font=("Times New Roman",12), width=6)
            b4.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_irradiation_time(100)))
            b4.bind("<ButtonRelease-1>", lambda e: release_button())
            b4.grid(row=1, column=0, padx=2, pady=2)
            
            b5 = tk.Button(incrementButtonFrame, text="+1000", font=("Times New Roman",12), width=6)
            b5.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_irradiation_time(1000)))
            b5.bind("<ButtonRelease-1>", lambda e: release_button())
            b5.grid(row=1, column=1, padx=2, pady=2)

            b6 = tk.Button(incrementButtonFrame, text="+10000", font=("Times New Roman",12), width=6)
            b6.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_irradiation_time(10000)))
            b6.bind("<ButtonRelease-1>", lambda e: release_button())
            b6.grid(row=1, column=2, padx=2, pady=2)

            b7 = tk.Button(incrementButtonFrame, text="-0.1", font=("Times New Roman",12), width=6)
            b7.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_irradiation_time(-0.1)))
            b7.bind("<ButtonRelease-1>", lambda e: release_button())
            b7.grid(row=2, column=0, padx=2, pady=2)

            b8 = tk.Button(incrementButtonFrame, text="-1", font=("Times New Roman",12), width=6)
            b8.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_irradiation_time(-1)))
            b8.bind("<ButtonRelease-1>", lambda e: release_button())
            b8.grid(row=2, column=1, padx=2, pady=2)

            b9 = tk.Button(incrementButtonFrame, text="-10", font=("Times New Roman",12), width=6)
            b9.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_irradiation_time(-10)))
            b9.bind("<ButtonRelease-1>", lambda e: release_button())
            b9.grid(row=2, column=2, padx=2, pady=2)

            b10 = tk.Button(incrementButtonFrame, text="-100", font=("Times New Roman",12), width=6)
            b10.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_irradiation_time(-100)))
            b10.bind("<ButtonRelease-1>", lambda e: release_button())
            b10.grid(row=3, column=0, padx=2, pady=2)

            b11 = tk.Button(incrementButtonFrame, text="-1000", font=("Times New Roman",12), width=6)
            b11.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_irradiation_time(-1000)))
            b11.bind("<ButtonRelease-1>", lambda e: release_button())
            b11.grid(row=3, column=1, padx=2, pady=2)

            b12 = tk.Button(incrementButtonFrame, text="-10000", font=("Times New Roman",12), width=6)
            b12.bind("<ButtonPress-1>", lambda e: press_and_hold(lambda: update_irradiation_time(-10000)))
            b12.bind("<ButtonRelease-1>", lambda e: release_button())
            b12.grid(row=3, column=2, padx=2, pady=2)
        
        

        keyboardButton = tk.Button(
            incrementButtonFrame,
            command=increment_buttons,  # Display keyboard(+-) buttons
            text="⌨",
            font=("Times New Roman", 16),
            bg=manualWindowBG,
            borderwidth=0,
        )
        keyboardButton.grid(row=0, column=0, columnspan=1, padx=1, pady=1)

        if irradiationTimeTK:
            irradiationTimeTK.focus_set()


        manualWindowStatus=True
        manualButtonsDisabled=True
    except Exception as e:
        safe_showerror("Error",f"Failed to open Manual Window:\n{e}")
        goBack()

def manual_run_command(k=-1,processType="Initializing..."):     #Needs to be global - updated from update_ui()
    """
    Updates the labels and buttons on the manual window according to the command

    """
    global manualWindowProcessLabel,manualRemainingTimeTK,totalTime,displayZigZag,irradiationTimeTK
    pseudo_garbage_collector()
    zigzagK= k+1 if (k//8)%2==0 else (k//8)*8 + (8 - k%8)
    displayZigZag=zigzagK       # Don't forget to delete other texts related to processtype !
    manualWindowProcessLabel.config(text=" ")
    manualRemainingTimeTK.config(text=" ")
    manualWindow_disable_all()    

    if processType=="Move":
        instruction_handler("Manual", positions[k // 8][k % 8])

    elif processType=="Irradiate Sample":
        instruction_handler("Irradiate",activationTimes=irradiationTimeTK.get()) #irradiationTimeTK still string, handled in instruction_handler
    elif processType=="Withdrawal":
        instruction_handler("Withdrawal")
    else:
        processType="forceHoming"
        instruction_handler("forceHoming")

    manualRemainingTimeTK.grid(row=1, column=0, padx=5, pady=5)

    manualWindow_disable_all()
    root.after(100, lambda: manual_decrement_timer(0, processType))
def manual_decrement_timer(startTime=0, processType=None):#needs to be global for manual_run_command
    global timeOutStatus, sentQueue, instructionQueue, manualRemainingTimeTK, manualWindowProcessLabel,totalTime
    try:
        if simulationQueue and startTime != 0:
            # Calculate elapsed time
            currentTime = time.time()
            elapsedTime = currentTime - startTime
            totalRemainingTime = totalTime - elapsedTime

            # Update labels
        if simulationQueue:
            if startTime == 0:
                startTime = time.time() # Initialize the start time once
                totalRemainingTime=totalTime
            manualRemainingTimeTK.config(text=f"Remaining Time: {int(totalRemainingTime)} seconds")
            root.after(500, lambda: manual_decrement_timer(startTime,  processType))
        
        elif stopStatus:
            manualRemainingTimeTK.config(text="")
            manualWindowProcessLabel.config(text="Stopping")
            return
        elif not simulationQueue:
            # Clear labels if simulation queue is empty

            #last update, do not delete !
            currentTime = time.time()
            if startTime==0:
                startTime = currentTime
            elapsedTime = currentTime - startTime
            totalRemainingTime = totalTime - elapsedTime

            manualRemainingTimeTK.config(text="")
            manualWindowProcessLabel.config(text="")
            run_time_check(totalRemainingTime)
            return
    except Exception as e:
        print(f"Early exit? :{e}")

def manualWindow_disable_all():
    """Disable sample buttons in the manual window and others if applicable"""
    global sampleButtons, homeButton, irradiateButton, withdrawalButton, manualAbortButton, manualStopButton, manualButtonsDisabled
    if manualButtonsDisabled:
        return
    if manualWindowStatus:
        for idx, (canvas, circle_id, text_id) in enumerate(sampleButtons):
            if idx == currentSampleIndex:
                # Highlight the selected button as disabled (green circle)
                canvas.itemconfig(circle_id, fill="green")  # Change circle color to green
                canvas.itemconfig(text_id,  fill="white")  # Bold white text
            else:
                # Disable unselected buttons (gray circle)
                canvas.itemconfig(circle_id, fill="gray")  # Reset circle color
                canvas.itemconfig(text_id,  fill="black")  # Reset text

            # Disable button interaction by unbinding the click event
            canvas.unbind("<Button-1>")

        # Disable other control buttons
        homeButton.config(state="disabled")
        withdrawalButton.config(state="disabled")
        irradiateButton.config(state="disabled")

        # Manual Abort and Stop buttons remain active depending on the status
        if connectionStatus == False or stopStatus == True:
            manualAbortButton.config(state="normal")
            manualStopButton.config(state="normal")

        # Set manualButtonsDisabled flag
        manualButtonsDisabled = True

def manualWindow_enable_all():
    """Enable sample buttons in the manual window and others if applicable"""
    global manualAbortButton,manualStopButton,manualButtonsDisabled
    global sampleButtons,homeButton,irradiateButton,withdrawalButton
    
    if not manualButtonsDisabled:
        return
    if manualWindowStatus:
        for idx, (canvas, circle_id, text_id) in enumerate(sampleButtons):
            if idx == currentSampleIndex:
                # Highlight the selected button
                canvas.itemconfig(circle_id, fill="green")  # Change circle color
                canvas.itemconfig(text_id, fill="white")  # Change text
            else:
                # Reset unselected buttons
                canvas.itemconfig(circle_id, fill="gray")  # Reset circle color
                canvas.itemconfig(text_id, fill="white")  # Reset text
            # Enable other buttons
            canvas.bind("<Button-1>", lambda e, k=idx: manual_run_command(k, "Move"))
        homeButton.config(state="normal")
        withdrawalButton.config(state="normal")
        manualAbortButton.config(state="normal")
        manualStopButton.config(state="disabled")
        irradiateButton.config(state="normal" if indexMatchFound and currentSampleIndex>=0 and not simulationQueue else "disabled")

        # Set the flag
        manualButtonsDisabled = False
        

def open_configuration_window():
    """"Stepper configuration window"""
    global maxSpeed, acceleration,unMatchedResponsesTK,configurationWindowStatus,currentPositionTK,sentQueueTK,instructionQueueTK,configurationWindow
    global abortStatus, stopStatus
    configurationWindowStatus=True # prioritized to prevent duplication
    
    def goBack():
        global configurationWindowStatus
        configurationWindowStatus=False
        configurationWindow.destroy()
        
    def choose_color():
        global masterColor
        color_code = colorchooser.askcolor(title="Choose a color")
        if color_code[1]:
            masterColor=str(color_code[1])
            colorButtonTK.config(bg=masterColor)
        
        
    #input validators
    def validate_input(value):
        if value == "":
            return True  
        try:
            number = int(value)
            return 999999 >= number > 0 
        except ValueError:
            return False  
    def validate_step(value):
        if value == "":
            return True  
        try:
            number = int(value)
            return 999999>number>0
        except ValueError:
            return False  

    def validate_speed(value):
        if value == "":
            return True  
        try:
            number = int(value)
            return 10001> number > 0
        except ValueError:
            return False  

    def validate_withdrawal_input(value):
        if value == "" or value == "-":
            return True  
        try:
            number = int(value)
            if number>100000 or number<-100000:
                return False
            return number  
        except ValueError:
            return False  
        
    def validate_guiScale(value):
        if value == "":
            return True  
        try:
            number = float(value)
            if number>10 or number<0.1:
                return False
            return number  
        except ValueError:
            return False  



    def send_command():
        try:
            x = int(x_position.get().strip())  # Validate and strip input
            y = int(y_position.get().strip())
            instruction_handler("Manual", (x, y))  # Call instruction handler
        except ValueError:
            safe_showerror("Invalid Input", "Please enter valid numeric values for X and Y positions.")

    # Create the configuration window
    configurationWindow = tk.Toplevel(root)
    configurationWindow.resizable(False, False)
    icon = PhotoImage(file=relative_to_assets("image_2.png"))
    configurationWindow.iconphoto(False, icon)
    configurationWindow.title("Configuration Window")
    configurationWindow.protocol("WM_DELETE_WINDOW", goBack)
    configurationWindow.grid_columnconfigure(0, weight=2)
    configurationWindow.grid_columnconfigure(1, weight=1)
    commandWindow = tk.Frame(configurationWindow)
    commandWindow.grid(row=0, column=0, padx=10, pady=10)

    timeoutFrame= tk.Frame(commandWindow)
    timeoutFrame.grid(row=0, column=0, padx=5, pady=1, sticky="ew") 

    withdrawalFrame = tk.Frame(commandWindow)
    withdrawalFrame.grid(row=1, column=0, padx=5, pady=1, sticky="ew") 
    withdrawalFrame.grid_columnconfigure(0,weight=0)
    withdrawalFrame.grid_columnconfigure(1,weight=0)
    withdrawalFrame.grid_columnconfigure(2,weight=0)

    guiScaleFrame= tk.Frame(commandWindow)
    guiScaleFrame.grid(row=2, column=0, padx=5, pady=1)
    guiScaleFrame.grid_columnconfigure(0,weight=0)
    guiScaleFrame.grid_columnconfigure(1,weight=0)
    guiScaleFrame.grid_columnconfigure(2,weight=0)

    positionFrame = tk.Frame(commandWindow)
    positionFrame.grid(row=3, column=0, padx=5, pady=1, sticky="ew")

    positionFrameFirstColumn=tk.Frame(positionFrame)
    positionFrameFirstColumn.grid(row=0, column=0, padx=5, pady=1, sticky="ew")
    positionFrameSecondColumn=tk.Frame(positionFrame)
    positionFrameSecondColumn.grid(row=0, column=1, padx=5, pady=1, sticky="ew")
    positionFrameThirdColumn= tk.Frame(positionFrame)
    positionFrameThirdColumn.grid(row=0, column=2, padx=5, pady=1, sticky="ew")

    buttonFrame = tk.Frame(commandWindow)
    buttonFrame.grid(row=4, column=0, padx=5, pady=1, sticky="ew") 

    bottomFrame = tk.Frame(commandWindow)
    bottomFrame.grid(row=5, column=0, padx=5, pady=1, sticky="ew") 

    queWindow = tk.Frame(configurationWindow)
    queWindow.grid(row=0, column=1, padx=20, pady=20, sticky="ew")

    # Entry validation commands
    scmd=(configurationWindow.register(validate_speed), '%P')
    vcmd=(configurationWindow.register(validate_input), '%P')
    vscmd=(configurationWindow.register(validate_step), '%P')
    wcmd=(configurationWindow.register(validate_withdrawal_input), '%P')
    scalecmd=(configurationWindow.register(validate_guiScale), '%P')
    
    # All buttons and entries
    tk.Label(timeoutFrame, text="Acceleration (step/s²):", font=("Times New Roman", 12)).grid(row=1, column=0, padx=10, pady=5, sticky="e")
    accelerationTK = tk.Entry(timeoutFrame, font=("Times New Roman", 12),validate="key",justify="center",validatecommand=scmd,borderwidth=2, width=6)
    accelerationTK.grid(row=1, column=1, padx=10, pady=5,sticky="n")
    accelerationTK.insert(0,str(acceleration))

    tk.Label(timeoutFrame, text="Max Speed (step/s):", font=("Times New Roman", 12)).grid(row=2, column=0, padx=10, pady=5, sticky="e")
    maxSpeedTK = tk.Entry(timeoutFrame, font=("Times New Roman", 12),validate="key",justify="center",validatecommand=scmd,borderwidth=2, width=6)
    maxSpeedTK.grid(row=2, column=1, padx=10, pady=5,sticky="n")
    maxSpeedTK.insert(0,str(maxSpeed))

    tk.Label(timeoutFrame, text="General Timeout (s):", font=("Times New Roman", 12)).grid(row=3, column=0, padx=10, pady=5, sticky="e")
    generalTimeOutTK = tk.Entry(timeoutFrame, font=("Times New Roman", 12),validate="key",justify="center",validatecommand=vcmd,borderwidth=2, width=6)
    generalTimeOutTK.grid(row=3, column=1, padx=10, pady=5,sticky="n")
    generalTimeOutTK.insert(0,str(generalTimeOut))

    tk.Label(timeoutFrame, text="Withdrawal Timeout (s):", font=("Times New Roman", 12)).grid(row=4, column=0, padx=10, pady=5, sticky="e")
    withdrawalTimeOutTK = tk.Entry(timeoutFrame, font=("Times New Roman", 12),validate="key",justify="center",validatecommand=vcmd,borderwidth=2, width=6)
    withdrawalTimeOutTK.grid(row=4, column=1, padx=10, pady=5,sticky="n")
    withdrawalTimeOutTK.insert(0,str(withdrawalTimeOut))

    tk.Label(timeoutFrame, text="Homing Timeout (s):", font=("Times New Roman", 12)).grid(row=5, column=0, padx=10, pady=5, sticky="e")
    homingTimeOutTK = tk.Entry(timeoutFrame, font=("Times New Roman", 12),validate="key",justify="center",validatecommand=vcmd,borderwidth=2, width=6)
    homingTimeOutTK.grid(row=5, column=1, padx=10, pady=5,sticky="n")
    homingTimeOutTK.insert(0,str(homingTimeOut))

    tk.Label(timeoutFrame, text="Ping Interval (s):", font=("Times New Roman", 12)).grid(row=6, column=0, padx=10, pady=5, sticky="e")
    pingIntervalTK = tk.Entry(timeoutFrame, font=("Times New Roman", 12),validate="key",justify="center",validatecommand=vcmd,borderwidth=2, width=6)
    pingIntervalTK.grid(row=6, column=1, padx=10, pady=5,sticky="n")
    pingIntervalTK.insert(0,str(pingInterval))
    
    tk.Label(timeoutFrame, text="Home After Steps (steps):", font=("Times New Roman", 12)).grid(row=7, column=0, padx=10, pady=5, sticky="e")
    homeAfterStepsTK = tk.Entry(timeoutFrame, font=("Times New Roman", 12),validate="key",justify="center",validatecommand=vscmd,borderwidth=2, width=6)
    homeAfterStepsTK.grid(row=7, column=1, padx=10, pady=5,sticky="n")
    homeAfterStepsTK.insert(0,str(homeAfterSteps))

    tk.Label(withdrawalFrame, text="Withdrawal Position(x,y):", font=("Times New Roman", 12)).grid(row=0, column=0, padx=10, pady=5, sticky="e")
    withdrawalPositionxTK = tk.Entry(withdrawalFrame, font=("Times New Roman", 12), validate="key", 
                                     validatecommand=wcmd, width=6,justify="center",borderwidth=2)
    withdrawalPositionxTK.grid(row=0, column=1, padx=8,sticky="n")  # Add small padding between entries
    withdrawalPositionxTK.insert(0, withdrawalPosition[0])

    withdrawalPositionyTK = tk.Entry(withdrawalFrame, font=("Times New Roman", 12), validate="key", validatecommand=wcmd, width=6,justify="center",borderwidth=2)
    withdrawalPositionyTK.grid(row=0, column=2,padx=0,sticky="n")
    withdrawalPositionyTK.insert(0, str(withdrawalPosition[1]))

    tk.Label(guiScaleFrame, text="           Gui Scale: ", font=("Times New Roman", 12)).grid(row=0, column=0, padx=11, pady=1, sticky="e")
    guiScaleTK= tk.Entry(guiScaleFrame, font=("Times New Roman", 12),validate="key", validatecommand=scalecmd,justify="center",borderwidth=2,width=6)
    guiScaleTK.grid(row=0,column=1,pady=1, padx=1, sticky="n")
    guiScaleTK.insert(0, guiScale)
    
    colorButtonTK=tk.Button(positionFrameFirstColumn, text="BG Color",font=("Times New Roman", 12), command=lambda:choose_color(),bg=masterColor,fg="black")
    colorButtonTK.grid(row=0,column=1, pady=1, padx=1,sticky="e")

    tk.Button(
    positionFrameSecondColumn,
    text="Save",
    font=("Times New Roman", 12),
    command=lambda: write_config_file(
        accelerationTK.get(),
        maxSpeedTK.get(),
        generalTimeOutTK.get(),
        withdrawalTimeOutTK.get(),
        homingTimeOutTK.get(),
        pingIntervalTK.get(),
        homeAfterStepsTK.get(),
        (int(withdrawalPositionxTK.get()), int(withdrawalPositionyTK.get())),
        guiScaleTK.get(),
        masterColor
        )
    ).grid(row=0, column=2,pady=1, padx=1)

    tk.Button(positionFrameThirdColumn, text="Set Position Map", font=("Times New Roman", 12),
    command=lambda: setup_positions(configurationWindow) if not setupPositionWindowStatus else None
    ).grid(row=0, column=3 , pady=1, padx=1)
    
    tk.Label(buttonFrame, text="             X Position(step):  ", font=("Times New Roman", 12)).grid(row=1, column=0, padx=10, pady=5, sticky="e")
    x_position = tk.Entry(buttonFrame, font=("Times New Roman", 12),validate="key",validatecommand=vcmd,width=8,justify="center",borderwidth=2)
    x_position.grid(row=1, column=1, padx=1, pady=1)
    x_position.insert(0,"100")

    tk.Label(buttonFrame, text="             Y Position(step):  ", font=("Times New Roman", 12)).grid(row=2, column=0, padx=10, pady=5, sticky="e")
    y_position = tk.Entry(buttonFrame, font=("Times New Roman", 12),validate="key",validatecommand=vcmd,width=8,justify="center",borderwidth=2)
    y_position.grid(row=2, column=1, padx=1, pady=1)
    y_position.insert(0,"100")
                                                            
    tk.Label(bottomFrame,text="         ").grid(row=0, column=0, padx=10, pady=5, sticky="e")# some padding to centralize
    tk.Button(bottomFrame, text="Send", font=("Times New Roman", 12),width=5,
              command=send_command).grid(row=0, column=4, columnspan=2, pady=5, padx=5)
    
    tk.Button(bottomFrame, text="Stop", font=("Times New Roman", 12),width=5,
              command=lambda: instruction_handler("Stop")).grid(row=0, column=6, columnspan=2, pady=10, padx=5)
    
    tk.Button(bottomFrame, text="Abort",font=("Times New Roman", 12),bg="red",fg="white",width=5,
               command=lambda: instruction_handler("Abort")).grid(row=0, column=8, columnspan=2, pady=10, padx=5)
    
    currentPositionTK=tk.Label(queWindow, text=f"Last Position:\n{currentPosition}\n",font=("Times New Roman", 12), height=5, width=20)
    currentPositionTK.grid(row=0, column=0, columnspan=2, padx=10, pady=5)

    # List for unmatched responses
    tk.Label(queWindow, text="Unmatched Responses", font=("Times New Roman", 12)).grid(row=1, column=0, padx=10, pady=5, sticky="e")
    unMatchedResponsesTK = tk.Listbox(queWindow ,font=("Times New Roman", 12), height=15, width=30)
    unMatchedResponsesTK.grid(row=2, column=0, columnspan=2, padx=10, pady=5)
    
    #updating the displays
    if(unMatchedResponses):
        for response in unMatchedResponses:
            unMatchedResponsesTK.insert(tk.END,str(response))

    sentQueueTK=tk.Label(queWindow, text=f"Sent Queue:      \n{sentQueue}      \n", font=("Times New Roman", 12))
    sentQueueTK.grid(row=0, column=5, padx=10, pady=5, sticky="e")

    tk.Label(queWindow, text="Instruction Queue  ", font=("Times New Roman", 12)).grid(row=1, column=5, padx=10, pady=5, sticky="e")
    instructionQueueTK = tk.Listbox(queWindow ,font=("Times New Roman", 12), height=15, width=30)
    instructionQueueTK.grid(row=2, column=5, columnspan=2, padx=10, pady=5)

    if(instructionQueue):
        for response in instructionQueue:
            instructionQueueTK.insert(tk.END,str(response))
    
def setup_positions(configurationWindow):
    """Sets up a position map with customization options."""
    global setupPositionWindowStatus
    setupPositionWindowStatus = True

    def goBack():
        global setupPositionWindowStatus
        setupPositionWindowStatus = False
        setupPositionWindow.destroy()

    # Default values for the matrix
    x_offset = tk.IntVar(value=0)
    y_offset = tk.IntVar(value=0)
    increment = tk.IntVar(value=950)

    # Initialize the matrix
    tempPositions = positions

    def generate_matrix(x_offset=4800, y_offset=4800, increment=-657, rows=8, cols=8):
        matrix = []
        for row in range(rows):
            matrix_row = []
            for col in range(cols):
                x = x_offset + col * increment
                y = y_offset + row * increment
                matrix_row.append((x, y))
            matrix.append(matrix_row)
        return matrix


    def update_matrix():
        """Updates the matrix and refreshes the displayed position map."""
        nonlocal tempPositions
        try:
            tempPositions = generate_matrix(
                increment=increment.get(),
                x_offset=x_offset.get(),
                y_offset=y_offset.get()
            )
            refresh_display()
        except ValueError:
            messagebox.showerror("Error", "Invalid input for offset or increment!")

    def refresh_display():
        """Refreshes the displayed position map."""
        for widget in mapFrame.winfo_children():
            widget.destroy()

        for r, row in enumerate(tempPositions):
            for c, pos in enumerate(row):
                # Display each position as a button for editing
                tk.Button(
                    mapFrame,text=f"({pos[0]}, {pos[1]})",
                    width=10,height=2,background="white",
                    command=lambda r=r, c=c: edit_position(r, c)
                ).grid(row=r, column=c, padx=5, pady=5)

    def edit_position(row, col):
        """Allows editing a specific position."""
        def save_edit():
            try:
                x = int(x_entry.get())
                y = int(y_entry.get())
                tempPositions[row][col] = (x, y)
                editWindow.destroy()
                refresh_display()
            except ValueError:
                messagebox.showerror("Error", "Invalid input for X or Y value!")

        editWindow = tk.Toplevel(setupPositionWindow)
        editWindow.title("Edit tempPosition")
        tk.Label(editWindow, text="X:").grid(row=0, column=0)
        x_entry = tk.Entry(editWindow)
        x_entry.grid(row=0, column=1)
        x_entry.insert(0, positions[row][col][0])

        tk.Label(editWindow, text="Y:").grid(row=1, column=0)
        y_entry = tk.Entry(editWindow)
        y_entry.grid(row=1, column=1)
        y_entry.insert(0, positions[row][col][1])

        tk.Button(editWindow, text="Save to file", command=save_edit).grid(row=2, column=0, columnspan=2)

    # Setup main window
    setupPositionWindow = tk.Toplevel(configurationWindow)
    setupPositionWindow.resizable(False, False)
    setupPositionWindow.title("Position Setup Window")
    setupPositionWindow.protocol("WM_DELETE_WINDOW", goBack)
    icon = PhotoImage(file=relative_to_assets("image_2.png"))
    setupPositionWindow.iconphoto(False, icon)

    # Input fields for x_offset, y_offset, and increment
    inputFrame = tk.Frame(setupPositionWindow)
    inputFrame.pack(pady=10)

    tk.Label(inputFrame, text="X Offset:").grid(row=0, column=0)
    tk.Entry(inputFrame, textvariable=x_offset).grid(row=0, column=1)
    tk.Label(inputFrame, text="Y Offset:").grid(row=1, column=0)
    tk.Entry(inputFrame, textvariable=y_offset).grid(row=1, column=1)
    tk.Label(inputFrame, text="Increment:").grid(row=2, column=0)
    tk.Entry(inputFrame, textvariable=increment).grid(row=2, column=1)

    tk.Button(inputFrame, text="Generate Map", command=update_matrix).grid(row=3, column=0, columnspan=2, pady=5)

    # Frame for position map
    mapFrame = tk.Frame(setupPositionWindow)
    mapFrame.pack(pady=10)

    # Save button
    tk.Button(setupPositionWindow, text="Save to File", command=lambda:write_position_file(tempPositions)).pack(pady=5)

    # Generate initial matrix
    refresh_display()


def initialize_root():
    """Main window"""
    global rootStatus, homingStatus, root, guiScale, rootConnectionLabel,screenHeight,screenWidth
    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", exit)
    read_config_file()
    read_position_file()
    rootBG = masterColor
    # Get screen size
    screenWidth = root.winfo_screenwidth()
    screenHeight = root.winfo_screenheight()
    root.tk.call("tk", "scaling", guiScale)  # Updates the fonts of some of the widgets
    rootScale= guiScale*0.9
    window_width = int(800 * rootScale)
    window_height = int(500 * rootScale)
    rootWindowOffset=f"+{int((screenWidth / 2) - (window_width / 2))}+{int((screenHeight / 2) - (window_height / 2)-50)}"
    root.geometry(f"{window_width}x{window_height}{rootWindowOffset}")
    root.update_idletasks()
    root.state('normal')
    root.attributes('-fullscreen', False)

    try:
       
        button1_raw = Image.open(relative_to_assets("button_1.png"))
        button1_scaled = button1_raw.resize(
            (int(button1_raw.width * rootScale), int(button1_raw.height * rootScale)),
            Image.Resampling.LANCZOS
        )
        button_image_1 = ImageTk.PhotoImage(button1_scaled)

        button2_raw = Image.open(relative_to_assets("button_2.png"))
        button2_scaled = button2_raw.resize(
            (int(button2_raw.width * rootScale), int(button2_raw.height * rootScale)),
            Image.Resampling.LANCZOS
        )
        button_image_2 = ImageTk.PhotoImage(button2_scaled)

        logo_raw = Image.open(relative_to_assets("image_2.png"))
        logo_scaled = logo_raw.resize(
            (int(logo_raw.width * rootScale), int(logo_raw.height * rootScale)),
            Image.Resampling.LANCZOS
        )
        image_2 = ImageTk.PhotoImage(logo_scaled)

    except Exception as e:
        safe_showerror(
            "Error",
            "Could not find or load assets at: " + relative_to_assets("") + "\n" + str(e)
        )
        return
    xOffset = -60
    # Canvas fills the window
    canvas = Canvas(
        root,
        bg=rootBG,
        width=window_width-1,
        height=window_height-1,
        bd=0,
        highlightthickness=0,
        relief="ridge"
    )
    canvas.place(x=0, y=0)
    canvas.update()

    # Gear button
    gear_button = tk.Button(
    root,
    text="⚙",          # Unicode gear symbol
    font=("segoe ui", int(13 * rootScale)),  # font that supports it
    borderwidth=0,
    highlightthickness=0,
    bg=rootBG,
    fg="black",
    command=lambda: open_configuration_window() if not configurationWindowStatus else None,
    relief="flat"
    )
    
    gear_button.place(
        x=int(8 * rootScale - 5),
        y=int(460 * rootScale - 10 * rootScale),
        width=int(50 * rootScale),
        height=int(50 * rootScale)
    )

    # Button 1
    button_1 = Button(
        image=button_image_1,
        borderwidth=0,
        highlightthickness=0,
        command=open_setup_window,
        relief="flat"
    )
    button_1.place(
        x=int(367 * rootScale+xOffset),
        y=int(285 * rootScale),
        width=int(128 * rootScale),
        height=int(41 * rootScale)
    )

    # Button 2
    button_2 = Button(
        image=button_image_2,
        borderwidth=0,
        highlightthickness=0,
        command=open_manual_window,
        relief="flat"
    )
    button_2.place(
        x=int(367 * rootScale+xOffset),
        y=int(356 * rootScale),
        width=int(128 * rootScale),
        height=int(41 * rootScale)
    )

    # Canvas text, scaled position & font sizes
    canvas.create_text(
        int(247 * rootScale+xOffset),
        int(11 * rootScale),
        anchor="nw",
        text="T",
        fill="#CC0000",
        font=("CroissantOne Regular", int(128 * rootScale * -1))
    )

    canvas.create_text(
        int(467 * rootScale+xOffset),
        int(11 * rootScale),
        anchor="nw",
        text="SL",
        fill="#CC0000",
        font=("CroissantOne Regular", int(128 * rootScale * -1))
    )

    canvas.create_text(
        int(247 * rootScale+xOffset),
        int(159 * rootScale),
        anchor="nw",
        text="ELSEC-9010 Custom Interface",
        fill="#CC0000",
        font=("Times New Roman", int(31 * rootScale * -1))
    )

    canvas.create_image(
        int(395 * rootScale+xOffset),
        int(85 * rootScale),
        image=image_2
    )

    # Connection label
    rootConnectionLabel = tk.Label(
        root,
        bg="green" if microcontrollerStatus else masterColor,
        text=f"{mcuID} - Not Connected" if not connectionStatus 
        else f"{mcuID} - Ready" if microcontrollerStatus 
        else f"{mcuID} - Busy",
        font=("Times New Roman", int(11 * rootScale), "bold"),
        justify="center",
        fg="black",
        width=16
    )
    rootConnectionLabel.place(
        x=window_width - int(5 * rootScale),
        y=window_height - int(5 * rootScale),
        anchor="se"
    )

    # App icon
    icon = ImageTk.PhotoImage(logo_scaled)
    time.sleep(0.01)
    root.iconphoto(False, icon)
    root.title("ELSEC-9010 Custom Interface")

    root.resizable(False, False)
    rootStatus = True

    update_ui()
    root.mainloop()


port=find_microcontroller_port()

if(port!=None):
    try:
        ser = serial.Serial(port, 9600, timeout=5)  
        offlineMode=False                                                          
        IOthread=threading.Thread(target=instruction_queue_handler,daemon=True)  # IOthread for Input/Output commands
        IOthread.start()
        thread_running=True
    except:
        tk.messagebox.showerror("Error", "Serial is busy. Please try again later.")
else:
    tk.messagebox.showerror("Error", "Microcontroller not found. Starting in offline mode.")
    offlineMode=True
    print("Failed to establish connection")
    IOthread=threading.Thread(target=instruction_queue_handler,daemon=True)  
    IOthread.start()
    thread_running=True
initialize_root()

stop_thread()
sys.exit()
