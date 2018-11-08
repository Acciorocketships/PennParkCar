import time

from Message import Message

message = Message()

while True:
    time.sleep(0.1)
    message.recieve()
    print(message.gyroz)
