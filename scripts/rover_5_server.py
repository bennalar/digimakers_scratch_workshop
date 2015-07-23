#! /usr/bin/env python

import serial
import time
import multiprocessing
import sys
import subprocess

import RPIO
import RPIO.PWM

BLUETOOTH_SERIAL_PORT = "/dev/rfcomm0"
BLUETOOTH_SERIAL_BAUD_RATE = 9600
MINI_DRIVER_SERIAL_PORT = "/dev/ttyUSB0"
MINI_DRIVER_SERIAL_BAUD_RATE = 9600

HW_RECORD_LEN = 7 # number of fields returned from the hardware device
HW_RECORD_FIELD_LEFT_ENCODER = 0
HW_RECORD_FIELD_RIGHT_ENCODER = 1
HW_RECORD_FIELD_SONAR_DISTANCE_CM = 2
HW_RECORD_FIELD_FRONT_LEFT_IR = 3
HW_RECORD_FIELD_FRONT_RIGHT_IR = 4
HW_RECORD_FIELD_REAR_LEFT_IR = 5
HW_RECORD_FIELD_REAR_RIGHT_IR = 6

#Ensure that our record length is consistent with the number of fields
assert( HW_RECORD_LEN -1 == HW_RECORD_FIELD_REAR_RIGHT_IR)

#---------------------------------------------------------------------------------------------------
class ProcessBase( multiprocessing.Process ):
    def __init__( self ):
        multiprocessing.Process.__init__( self )
        self._stop = multiprocessing.Event()

    def stop( self ):
        self._stop.set()

    def stopped( self ):
        return self._stop.is_set()

#---------------------------------------------------------------------------------------------------
class ArtifactDetector( ProcessBase ):
    
    IDEAL_LOOP_TIME = 1.0/20.0  # Runs at 20Hz
    IMAGE_WIDTH = 320
    IMAGE_HEIGHT = 240
    IMAGE_JPEG_QUALITY = 85
    HORIZONTAL_FOV_DEGREES = 53.13
    
    MARKER_ID_TO_ARTIFACT_ID_MAP = {
        6 : 3,
        354 : 4,
        724 : 5,
        1014 : 6
    }
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, commandQueue, detectionResultQueue ):
        
        ProcessBase.__init__( self )
        
        self.commandQueue = commandQueue
        self.detectionResultQueue = detectionResultQueue
        self.curDetectionId = 0
        
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        while not self.stopped():
            
            loopStartTime = time.time()
            
            # Get the next command off the top of the queue if there is one
            if not self.commandQueue.empty():
                
                newCommand = self.commandQueue.get_nowait()
                
                # The command is always to detect an artifact so capture an image
                print "Starting detection"
                subprocess.call( [ "raspistill", "-w", "{0}".format( self.IMAGE_WIDTH ), 
                "-h", "240", "-hf", "-vf", "-o", "/home/pi/artifact_detect.jpg" ] )

                print "Photo taken"

                # Try to detect AR markers in the image
                detectionResult = subprocess.check_output( 
                    [ "sudo", "/home/pi/digimakers_scratch_workshop/ar_marker_detector/ar_marker_detector",
                    "/home/pi/artifact_detect.jpg" ], stderr=subprocess.PIPE )

                print "Detection run"
                print detectionResult
                    
                markerId = -1

                lines = detectionResult.split( "\n" )
                for line in lines:

                    try:
                        parts = line.split( "=" )
                        if len( parts ) >= 2:
                            
                            markerId = int( parts[ 0 ] )    # May raise parse exception
                            
                            coords = parts[ 1 ].split( " " )
                            
                            if len( coords ) >= 4:
                                
                                accX = 0.0
                                numCoordsParsed = 0
                                
                                for coordIdx in range( 4 ):
                                    
                                    xy = coords[ coordIdx ].split( "," )
                                    if len( xy ) >= 1:
                                        
                                        x = float( xy[ 0 ].strip( "(" ) )
                                        accX += x
                                        numCoordsParsed += 1
                                        
                                if numCoordsParsed == 4:
                                    
                                    markerCentreX = accX/4.0
                                    break                   # Don't bother to parse other lines
                    except:
                        pass    # Ignore errors that occur when parsing the line

                artifactFound = False
                if markerId in self.MARKER_ID_TO_ARTIFACT_ID_MAP:
                    
                    artifactId = self.MARKER_ID_TO_ARTIFACT_ID_MAP[ markerId ]
                    artifactFound = True
                        
                if not artifactFound:
                    artifactId = -1
                    markerCentreX = self.IMAGE_WIDTH/2.0

                markerHeadingDegrees = (markerCentreX - self.IMAGE_WIDTH/2.0)/(self.IMAGE_WIDTH/2.0) * (self.HORIZONTAL_FOV_DEGREES/2.0)
        
                result = ( self.curDetectionId, artifactId, markerHeadingDegrees )
                self.detectionResultQueue.put( result )
                self.curDetectionId += 1
                
            # Sleep if needed
            elapsedTime = time.time() - loopStartTime
            loopTimeRemaining = self.IDEAL_LOOP_TIME - elapsedTime
            
            if loopTimeRemaining > 0.0:
                time.sleep( loopTimeRemaining )
        
#---------------------------------------------------------------------------------------------------
class StateReporter( ProcessBase ):

    """Reads from motor driver and transmits status"""
    
    MAX_SONAR_DISTANCE_CM = 400
    SONAR_PIN = 24
    SONAR_TIMEOUT = 1.0
    IDEAL_LOOP_TIME = 1.0/20.0  # Runs at 20Hz
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, bluetoothSerial, miniDriverSerial, detectionResultQueue ):
        
        ProcessBase.__init__( self )
        
        self.bluetoothSerial = bluetoothSerial
        self.miniDriverSerial = miniDriverSerial
        self.detectionResultQueue = detectionResultQueue
        
        self.leftEncoderCount = 0
        self.rightEncoderCount = 0
        self.sonarDistanceCM = self.MAX_SONAR_DISTANCE_CM
        self.serialBuffer = ""
        self.lastDetectionId = -1
        self.detectedArtifactId = -1
        self.detectedArtifactHeadingDegrees = 0.0

    # #---------------------------------------------------------------------------------------------------
    # def readDistanceFromSonarCM( self ):
    #
    #     distance = 0.0
    #
    #     # Send the start signal
    #     RPIO.setup( self.SONAR_PIN, RPIO.OUT )
    #     RPIO.output( self.SONAR_PIN, RPIO.LOW )
    #     #time.sleep( 0.00002 )
    #
    #     RPIO.output( self.SONAR_PIN, RPIO.HIGH )
    #     time.sleep( 0.00001 )
    #     RPIO.output( self.SONAR_PIN, RPIO.LOW )
    #
    #     # Prepare to receive the response
    #     RPIO.setup( self.SONAR_PIN, RPIO.IN, pull_up_down=RPIO.PUD_DOWN )
    #
    #     # TODO: It would be nicer to use interrupts here, but I can't get them to work just yet...
    #     waitStartTime = time.time()
    #     pulseStartTime = time.time()
    #     while RPIO.input( self.SONAR_PIN ) == 0:
    #         curTime = time.time()
    #         pulseStartTime = curTime
    #
    #         if curTime - waitStartTime > self.SONAR_TIMEOUT:
    #             return self.MAX_SONAR_DISTANCE_CM
    #
    #     pulseEndTime = curTime = time.time()
    #     while RPIO.input( self.SONAR_PIN ) == 1:
    #         curTime = time.time()
    #         pulseEndTime = curTime
    #
    #         if curTime - waitStartTime > self.SONAR_TIMEOUT:
    #             return self.MAX_SONAR_DISTANCE_CM
    #
    #     measuredTimeUS = (pulseEndTime - pulseStartTime)*1000000.0
    #     distanceCM = int( measuredTimeUS / 58.0 )   # Using formula on http://www.seeedstudio.com/wiki/Ultra_Sonic_range_measurement_module
    #
    #     if distanceCM > self.MAX_SONAR_DISTANCE_CM:
    #         distanceCM = self.MAX_SONAR_DISTANCE_CM
    #
    #     return distanceCM

    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        while not self.stopped():
            
            loopStartTime = time.time()
            
            # Read status messages from the mini driver
            numBytesAvailable = self.miniDriverSerial.inWaiting()
            if numBytesAvailable > 0:
                
                self.serialBuffer += self.miniDriverSerial.read( numBytesAvailable )
                
                statusData = ""
                endOfLinePos = self.serialBuffer.find( "\n" )
                
                while endOfLinePos != -1:
                    
                    # Remove lines found in the serial data
                    statusData = self.serialBuffer[ :endOfLinePos ]
                    self.serialBuffer = self.serialBuffer[ endOfLinePos + 1: ]
                    
                    endOfLinePos = self.serialBuffer.find( "\n" )
                    
                # Extract the current status of the robot from the last line                
                statusItems = statusData.split()
                if len( statusItems ) >= HW_RECORD_LEN:
                    
                    try:
                        self.leftEncoderCount = int( statusItems[ HW_RECORD_FIELD_LEFT_ENCODER ] )
                        self.rightEncoderCount = int( statusItems[ HW_RECORD_FIELD_RIGHT_ENCODER ] )
                        self.sonarDistanceCM = int( statusItems[ HW_RECORD_FIELD_SONAR_DISTANCE_CM ] )
                        # self.irSensorFrontLeft = int( statusItems[ HW_RECORD_FIELD_FRONT_LEFT_IR ] )
                        # self.irSensorFrontRight = int( statusItems[ HW_RECORD_FIELD_FRONT_RIGHT_IR ] )
                        # self.irSensorRearLeft = int( statusItems[ HW_RECORD_FIELD_REAR_LEFT_IR ] )
                        # self.irSensorRearRight = int( statusItems[ HW_RECORD_FIELD_REAR_RIGHT_IR ] )
                    except:
                        pass    # Ignore parsing errors


            # Get the latest detection result if it exists
            if not self.detectionResultQueue.empty():
                detectionResult = self.detectionResultQueue.get_nowait()
                self.lastDetectionId, self.detectedArtifactId, self.detectedArtifactHeadingDegrees = detectionResult
            
            # Transmit the current status back
            #print "Writing", "{0} {1}\n".format( self.leftEncoderCount, self.rightEncoderCount )
            self.bluetoothSerial.write( "{0} {1} {2} {3} {4} {5}\n".format( 
                self.leftEncoderCount, self.rightEncoderCount, self.sonarDistanceCM,
                self.lastDetectionId, self.detectedArtifactId, self.detectedArtifactHeadingDegrees ) )
    
            # Sleep if needed
            elapsedTime = time.time() - loopStartTime
            loopTimeRemaining = self.IDEAL_LOOP_TIME - elapsedTime
            
            if loopTimeRemaining > 0.0:
                time.sleep( loopTimeRemaining )
    
#---------------------------------------------------------------------------------------------------
class CommandHandler( ProcessBase ):

    """Reads from bluetoothSerial and then controls the robot"""
    
    IDEAL_LOOP_TIME = 1.0/20.0  # Runs at 20Hz
    
    COMMAND_STATE_DICT = {
        "l" : "TurningLeft",
        "r" : "TurningRight",
        "f" : "DrivingForward",
        "b" : "DrivingBackward",
        "s" : "Stopped"
    }
    
    BUZZER_PIN = 8
    BUZZER_LED_PIN = 4
    BUZZER_TIME = 0.25
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, bluetoothSerial, miniDriverSerial, commandQueue ):
        
        ProcessBase.__init__( self )
        
        self.bluetoothSerial = bluetoothSerial
        self.miniDriverSerial = miniDriverSerial
        self.commandQueue = commandQueue
        
        self.buzzerActive = False
        self.buzzerStartTime = 0
        RPIO.setup( self.BUZZER_PIN, RPIO.OUT )
        RPIO.setup( self.BUZZER_LED_PIN, RPIO.OUT )
        
        self.roverState = "Stopped"
        
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        while not self.stopped():
            
            loopStartTime = time.time()
            
            # Read commands from bluetooth
            while self.bluetoothSerial.inWaiting():
                command = self.bluetoothSerial.read()
                print "Received command %s from bluetooth" % command
                if command in self.COMMAND_STATE_DICT:
                    self.roverState = self.COMMAND_STATE_DICT[ command ]
                elif command == "u":

                    self.buzzerActive = True
                    self.buzzerStartTime = time.time()
                elif command == "p":
                    
                    # Try to detect an artifact
                    self.commandQueue.put( command )
                
            # Use the current rover state to work out what to send to the mini driver
            if self.roverState == "Stopped":
                
                self.miniDriverSerial.write( "s" )
                
            elif self.roverState == "TurningLeft":
                
                self.miniDriverSerial.write( "l" )
                
            elif self.roverState == "TurningRight":
                
                self.miniDriverSerial.write( "r" )
            
            elif self.roverState == "DrivingForward":
                
                self.miniDriverSerial.write( "f" )
                
            elif self.roverState == "DrivingBackward":
                
                self.miniDriverSerial.write( "b" )
                
            else:

                # Should never get here, but switch back to stopped state if we do
                self.roverState = "Stopped"
                self.miniDriverSerial.write( "s" )

            # Update the buzzer
            if self.buzzerActive:
                if time.time() - self.buzzerStartTime > self.BUZZER_TIME:
                    self.buzzerActive = False

            if self.buzzerActive:

                # Buzzer on
                RPIO.output( self.BUZZER_PIN, RPIO.HIGH )
                RPIO.output( self.BUZZER_LED_PIN, RPIO.HIGH )

            else:

                # Buzzer off
                RPIO.output( self.BUZZER_PIN, RPIO.LOW )
                RPIO.output( self.BUZZER_LED_PIN, RPIO.LOW )
            
            # Sleep if needed
            elapsedTime = time.time() - loopStartTime
            loopTimeRemaining = self.IDEAL_LOOP_TIME - elapsedTime
            
            if loopTimeRemaining > 0.0:
                time.sleep( loopTimeRemaining )

#---------------------------------------------------------------------------------------------------
def cleanupProcesses( processes ):
    for process in processes:
        process.stop()

    for process in processes:
        process.join()
                
#---------------------------------------------------------------------------------------------------
if __name__ == '__main__':

    ## Setup RPIO, and prepare for PWM signals
    RPIO.setmode( RPIO.BCM )
    
    bluetoothSerial = serial.Serial( BLUETOOTH_SERIAL_PORT, baudrate=BLUETOOTH_SERIAL_BAUD_RATE, timeout=0 )
    miniDriverSerial = serial.Serial( MINI_DRIVER_SERIAL_PORT, baudrate=MINI_DRIVER_SERIAL_BAUD_RATE, timeout=0 )

    # Wait until we hear from the mini driver
    print "Waiting for connection to mini driver..."
    time.sleep( 12.0 )
    
    while miniDriverSerial.inWaiting() == 0:
        pass
    print "Mini driver connected!"

    # Create processes
    print "Starting up robot server..."
    
    commandQueue = multiprocessing.Queue()
    detectionResultQueue = multiprocessing.Queue()
    
    artifactDetector = ArtifactDetector( commandQueue, detectionResultQueue )
    stateReporter = StateReporter( bluetoothSerial, miniDriverSerial, detectionResultQueue )
    commandHandler = CommandHandler( bluetoothSerial, miniDriverSerial, commandQueue )

    artifactDetector.start()
    stateReporter.start()
    commandHandler.start()
    print "Robot server running! Enjoy..."
    
    while True:
        
        # Wait for a ctrl+c
        try:
            time.sleep( 0.1 )
            
        except KeyboardInterrupt:
            cleanupProcesses( ( artifactDetector, stateReporter, commandHandler ) )
            RPIO.cleanup()
            sys.exit()

                
