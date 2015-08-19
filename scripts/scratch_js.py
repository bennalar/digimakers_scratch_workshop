#!/usr/bin/python
# This code extends the existing mesh code to enable the new JavaScript scratch module to communicate with our robot/robot simulator.
# Rather than immediately broadcasting sensor values when they change. These robot controllers cache the sensor values and wait until
# a reporter code fraegment from scratch requests the sensor value. It is then returned back to the scratch JavaScript module.
import argparse
from websocket_server import WebsocketServer
import scratch_simulator_background_script
import multiprocessing
import logging
import scratch_rover_5_background_script
import py_websockets_bot

#Constants - should be shared amongst all python scripts
PREPARE_FOR_COMMAND_CMD = "prepareforcommand"
MODE_LIVE = 'live'
MODE_SIMULATOR = 'simulator'
RESET_CMD = "reset"
DETECT_ARTIFACT_CMD = "detectartifact"
MOVE_CMD = "move"
TURN_CMD = "turn"
ALL_CMDS_COMPLETE = "allCommandsComplete"
REPORTER_CMD = "reporter"
REPORTER_RESULT_CMD = "reporterResult"

#Hostname of the remote robot that we will control in the live mode
ROBOT_REMOTE_HOST = 'localhost'

#Port number that scratch will connect to
WEBSOCKETS_PORT_NUMBER = 42004

global logging

# Extends existing simulator to cache the sensor results
class JSRobotSimulator( scratch_simulator_background_script.RobotSimulator ):

    def __init__( self, commandQueue, reporterValuesQueue,  completedCommandsQueue):
        scratch_simulator_background_script.RobotSimulator.__init__(self, None, commandQueue)
        self.reporterValuesQueue = reporterValuesQueue
        self.completedCommandsQueue = completedCommandsQueue

    def sendSensorUpdate( self, reporterKey, value ):
        logging.debug("Sensor update: %s, %s"%(reporterKey, str(value)))
        #TODO - what happens if multiple commands are waiting from different scratch threads?
        if reporterKey == ALL_CMDS_COMPLETE:
            if value == 1:
                self.completedCommandsQueue.put((reporterKey, str(value)))
            #else ignore
        else:
            #else is a sensor/reporter value
            self.reporterValuesQueue.put((reporterKey, value))


# Extends existing controller to cache the sensor results
class JSRobotController( scratch_rover_5_background_script.RobotController):

    def __init__( self, commandQueue, reporterValuesQueue,  completedCommandsQueue, robot):
        scratch_rover_5_background_script.RobotController.__init__(self, None, commandQueue, robot )
        self.reporterValuesQueue = reporterValuesQueue
        self.completedCommandsQueue = completedCommandsQueue

    def sendSensorUpdate( self, reporterKey, value ):
        logging.debug("Sensor update: %s, %s"%(reporterKey, str(value)))
        #TODO - what happens if multiple commands are waiting from different scratch threads?
        if reporterKey == ALL_CMDS_COMPLETE:
            if value == 1:
                self.completedCommandsQueue.put((reporterKey, str(value)))
            #else ignore
        else:
            #else is a sensor/reporter value
            self.reporterValuesQueue.put((reporterKey, value))


def check_completed_commands(server):
    while not server.completedCommandsQueue.empty():
        (key, value) = server.completedCommandsQueue.get_nowait()
        logging.debug("Sending 'command completed' command to all clients")
        server.send_message_to_all("%s %s"%(key, str(value)))

def new_client(client, server):
    logging.debug("New client connected and was given id %d" % client['id'])

def client_left(client, server):
    logging.debug("Client(%d) disconnected" % client['id'])

def message_received(client, server, command):
    logging.debug("Client(%d) sent message: %s" % (client['id'], command))

    #check if reporter request
    #format of reporter request = reporter<name of variable>
    if command.startswith(REPORTER_CMD):
        #extract all items in the queue and place in dictionary to return on request
        while not server.reporterValuesQueue.empty():
            (key, value) = server.reporterValuesQueue.get_nowait()
            server.reporterValues[key] = value

        reporterKey = command[len(REPORTER_CMD):]
        if reporterKey in server.reporterValues:
            logging.debug("Sending reporter value %s: %s to client: %d" % (str(reporterKey), str(server.reporterValues[reporterKey]), client['id']))
            server.send_message(client, "%s %s %s"%(REPORTER_RESULT_CMD, str(reporterKey), str(server.reporterValues[reporterKey])))
    elif command.startswith(RESET_CMD):
        server.httpRobotSimulator.commandQueue.put( command )
    else:
        server.httpRobotSimulator.commandQueue.put( PREPARE_FOR_COMMAND_CMD )
        server.httpRobotSimulator.commandQueue.put( command )

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

    subparsers = parser.add_subparsers(dest='mode')
    simulatorParser = subparsers.add_parser(MODE_SIMULATOR)
    liveModeParser = subparsers.add_parser(MODE_LIVE)
    liveModeParser.add_argument(
        '--host',
        help="Remote Robot Hostname",
        dest="remoteRobotHostname",
        default=ROBOT_REMOTE_HOST,
        required=True,
    )

    args = parser.parse_args()
    logging.basicConfig(level=args.loglevel, format='%(asctime)s %(message)s')

    commandQueue = multiprocessing.Queue()
    reporterValuesQueue = multiprocessing.Queue()
    completedCommandsQueue = multiprocessing.Queue()

    server = WebsocketServer(WEBSOCKETS_PORT_NUMBER)
    server.reporterValues = {}
    server.reporterValuesQueue = reporterValuesQueue
    server.completedCommandsQueue = completedCommandsQueue
    server.set_fn_new_client(new_client)
    server.set_fn_client_left(client_left)
    server.set_fn_message_received(message_received)

    if args.mode == MODE_SIMULATOR:
        httpRobotSimulator = JSRobotSimulator(commandQueue, reporterValuesQueue, completedCommandsQueue)
    elif args.mode == MODE_LIVE:
        robot = py_websockets_bot.WebsocketsBot( args.remoteRobotHostname )
        httpRobotSimulator = JSRobotController(commandQueue, reporterValuesQueue, completedCommandsQueue, robot)
    else:
        raise "Unrecognised mode"

    server.httpRobotSimulator = httpRobotSimulator
    httpRobotSimulator.start()
    server.timeout = 0.1 #allows us to check if any commands have completed
    logging.info("Server is listening ")
    while 1:
        check_completed_commands(server)
        server.handle_request()