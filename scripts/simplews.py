__author__ = 'phil'

from websocket_server import WebsocketServer

def message_received(client, server, command):
    print("Client(%d) sent message: %s" % (client['id'], command))
    server.send_message_to_all(command)

def new_client(client, server):
    server.send_message_to_all("Hey all, a new client has joined us")

server = WebsocketServer(13254)
server.set_fn_new_client(new_client)
server.set_fn_message_received(message_received)
server.run_forever()