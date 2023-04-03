import time
import zmq

#  Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://10.104.17.232:5555")
print("Connecting to hello world server…")


#  Do 10 requests, waiting each time for a response

for request in range(10):
    print(f"Sending request {request} ...")
    socket.send_string("Hello")

    #  Get the reply.
    message = socket.recv()
    print(f"Received reply {request} [ {message} ]")