import paho.mqtt.client as mqtt
import ssl
import pprint
import json
from pymongo import MongoClient


MONGODB_IPADDRESS = '192.168.4.119'
MONGODB_PORTNUMBER = 27017

class RosTmsDbUpdate():
    def __init__(self):
        self.client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        self.db = self.client.rostmsdb
        self.collection = self.db.default
    
    def pozyx_update(self, position):
        cursor = self.collection.find({"id": 10004})
        document_array = []
        for doc in cursor:
            document_array.append(doc)
        pprint.pprint(document_array)
        return self.collection.update_one({"id": 10004},{"$set":{"position": position}})

dbUpdate = RosTmsDbUpdate()

host = "mqtt.cloud.pozyxlabs.com"
port = 443
topic = "5dede9522050afa070058bc5"  # your mqtt topic
username = "5dede9522050afa070058bc5"  # your mqtt username
password = "80fb1af1-00a3-4bbe-9d0a-f6db5e7b61b7"  # your generated api key

def on_connect(client, userdata, flags, rc):
    print(mqtt.connack_string(rc))

# Callback triggered by a new Pozyx data packet
def on_message(client, userdata, msg):
    datas = json.loads(msg.payload.decode())
    pp = pprint.PrettyPrinter(indent=4)
    for d in datas:
        if int(d["tagId"]) == 0x6969 and d["success"]: #28164 pc 
            #pp.pprint(d["data"]["coordinates"])
            position = d["data"]["coordinates"]
            position["x"] /= 1000.0
            position["y"] /= 1000.0
            position["z"] = 0.0
            pos = [position["x"], position["y"], position["z"]]
            print(pos)
            # pp.pprint(position)
            dbUpdate.pozyx_update(pos)
    # print("Positioning update:", msg.payload.decode())

def on_subscribe(client, userdata, mid, granted_qos):
    print("Subscribed to topic!")

client = mqtt.Client(transport="websockets")
client.username_pw_set(username, password=password)

# sets the secure context, enabling the WSS protocol
client.tls_set_context(context=ssl.create_default_context())

# set callbacks
client.on_connect = on_connect
client.on_message = on_message
client.on_subscribe = on_subscribe
client.connect(host, port=port)
client.subscribe(topic)

# works blocking, other, non-blocking, clients are available too.
client.loop_forever()
