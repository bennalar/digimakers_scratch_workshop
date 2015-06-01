#!/usr/bin/python
#This code just extends the existing mesh code for backwards compatibility
import argparse
import time
import BaseHTTPServer
import urlparse
import scratch_simulator_background_script
import multiprocessing
import logging
import re

#Constants - should be shared amongst all python scripts
PREPARE_FOR_COMMAND_CMD = "prepareforcommand"
RESET_CMD = "reset"
DETECT_ARTIFACT_CMD = "detectartifact"
MOVE_CMD = "move"
TURN_CMD = "turn"

HOST_NAME = 'localhost'
HTTP_PORT_NUMBER = 42002

#Turn on/off poll GET requet logs - we have around 25 a second
doNotLogPollMessages = True

#Only one instance of the server should be created
class ScratchHTTPServer:
    global httpRobotSimulator
    def __init__(self, (hostName, portNumber), httpRobotSim):
        handler = ScratchHTTPHandler
        ScratchHTTPServer.httpRobotSimulator = httpRobotSim
        ScratchHTTPHandler.httpRobotSimulator = httpRobotSim
        ScratchHTTPHandler.reporter_values = {}

        self.hostName = hostName
        self.portNumber = portNumber
        self.server = BaseHTTPServer.HTTPServer((hostName, portNumber), handler)
        
    def run(self):
        print time.asctime(), "Server Starts - %s:%s" % (self.hostName, self.portNumber)
        try:
            self.server.serve_forever()
        except KeyboardInterrupt:
            pass
        self.server.server_close()
        print time.asctime(), "Server Stops - %s:%s" % (self.hostName, self.portNumber)



# Handler to process requests from the scratch extension service
class ScratchHTTPHandler( BaseHTTPServer.BaseHTTPRequestHandler ):
    global httpRobotSimulator
    global reporter_values #Stores the reporter values

    def reset(self, params):
        self.submitCommand( RESET_CMD )
        ScratchHTTPHandler.reporter_values.clear()
    
    def poll(self, params):
        return self.checkForReporterUpdates()

    def forward(self, params):
        self.submitCommand( MOVE_CMD+ "%s" % params[0] )
    
    def reverse(self, params):
        self.submitCommand( MOVE_CMD+ "-%s" % params[0] )
    
    def left(self, params):
        self.submitCommand( TURN_CMD + "-%s" % params[0])
    
    def right(self, params):
        self.submitCommand( TURN_CMD + "%s" % params[0])
    
    def detectArtifact(self, params):
        self.submitCommand( DETECT_ARTIFACT_CMD )

    def do_HEAD(self):
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()

    def submitCommand(self, command):
        """Submit the command to the simulator."""
        ScratchHTTPHandler.httpRobotSimulator.commandQueue.put( PREPARE_FOR_COMMAND_CMD )
        ScratchHTTPHandler.httpRobotSimulator.commandQueue.put( command )

    def do_GET(self):
        """Respond to a scratch HTTP request."""
        
        cmds = {
            "poll" : self.poll,
            "forward" : self.forward,
            "reverse" : self.reverse,
            "left" : self.left,
            "right" : self.right,
            "detectArtifact" : self.detectArtifact,
            "reset_all" : self.reset,
        }
        parsed_path = urlparse.urlparse(self.path)
        message = ""
        cmdpath = parsed_path[2].split('/')
        handler = cmds[cmdpath[1]]
        pollResp = handler(cmdpath[2:])
        if cmdpath[1] == "poll":
            message_parts = []
            message_parts.append('')
            message_parts.append(pollResp)
            message = '\r\n'.join(message_parts)
        self.send_response(200)
        self.end_headers()
        self.wfile.write(message)
        return


    def checkForReporterUpdates(self):
        """Checks the reporter queue to see if any values have changed. Updates the dictionary if necessary"""

        message_parts = []
        while not ScratchHTTPHandler.httpRobotSimulator.reporterValuesQueue.empty():
            (key, value) = ScratchHTTPHandler.httpRobotSimulator.reporterValuesQueue.get_nowait()

            strKey = str(key)
            #Apply transformations to reporter values to convert from old block format to the new format
            match = re.match(r'artifact(X|Y|BearingDegrees)', strKey)
            if match:
                if match.group(1) == "BearingDegrees":
                    key = 'artifactPos/headingDegrees'
                else:
                    key = 'artifactPos/{0}'.format(match.group(1).lower())
            else:
                match = re.match(r'rover(X|Y|HeadingDegrees)', strKey)
                if match:
                    if match.group(1) == "HeadingDegrees":
                        key = 'roverPos/headingDegrees'
                    else:
                        key = 'roverPos/{0}'.format(match.group(1).lower())

                else:
                    match = re.match(r'Obstacle_([0-9])_(\w+)', strKey)
                    if match:
                        obstacleNumber = match.group(1)
                        obstacleParam = match.group(2)
                        if obstacleParam == "HeadingDegrees":
                            obstacleParam = 'headingDegrees'
                        else:
                            obstacleParam = obstacleParam.lower()
                        key = 'obstaclePos/{0}/{1}'.format(obstacleNumber, obstacleParam)

            ScratchHTTPHandler.reporter_values[key] = value

        for key,value in ScratchHTTPHandler.reporter_values.iteritems():
            reporter_txt = "{0} {1}".format(str(key), str(value))
            message_parts.append(reporter_txt)
            logging.debug('Sending back reporter key/value: %s', reporter_txt);

        return '\r\n'.join(message_parts)

    def log_request(self, code='-', size='-'):
        """Log an accepted request.

        This is called by send_response().

        """
        #Only log non-poll messages
        if not (doNotLogPollMessages and str(self.requestline).startswith('GET /poll ')):
            self.log_message('"%s" %s %s',
                            self.requestline, str(code), str(size))

# Extends existing simulator - this should hopefully be useful for the robot controller too
# Receives
class HTTPRobotSimulator( scratch_simulator_background_script.RobotSimulator ):

    def __init__( self, commandQueue, reporterValuesQueue):
        scratch_simulator_background_script.RobotSimulator.__init__(self, None, commandQueue)
        self.reporterValuesQueue = reporterValuesQueue

    def sendSensorUpdate( self, reporterKey, value ):
        """Add reporter value to dictionary"""

        self.reporterValuesQueue.put((reporterKey, value))


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-d', '--debug',
        help="Print lots of debugging statements",
        action="store_const", dest="loglevel", const=logging.DEBUG,
        default=logging.WARNING,
    )
    parser.add_argument(
        '-v', '--verbose',
        help="Be verbose",
        action="store_const", dest="loglevel", const=logging.INFO,
    )
    args = parser.parse_args()
    logging.basicConfig(level=args.loglevel, format='%(asctime)s %(message)s')

    commandQueue = multiprocessing.Queue()
    reporterValuesQueue = multiprocessing.Queue()
    httpRobotSimulator = HTTPRobotSimulator(commandQueue, reporterValuesQueue)

    httpd = ScratchHTTPServer((HOST_NAME, HTTP_PORT_NUMBER), httpRobotSimulator)

    httpRobotSimulator.start()
    httpd.run()


