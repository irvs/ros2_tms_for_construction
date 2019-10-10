from rclpy.node import Node
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData
import rclpy
import pymongo
import tms_db_manager.tms_db_util as db_util

client = pymongo.MongoClient("localhost:27017")
db = client.rostmsdb

class TmsDbReader(Node):
  
    def __init__(self):
        super().__init__('tms_db_reader')
        self.srv = self.create_service(TmsdbGetData, 'tms_db_reader', self.dbReaderSrvCallback)

    def dbReaderSrvCallback(self, request, response):
        response_tmsdb = Tmsdb()

        mode = 0
        temp_id = 0
        temp_type = ""
        temp_name = ""
        temp_place = 0
        sid = 100000

        MODE_ALL = 0
        MODE_NAME_IDTABLE = 1
        MODE_NAME = 2
        MODE_NAME_SENSOR = 3
        MODE_ID_IDTABLE = 4
        MODE_ID = 5
        MODE_ID_SENSOR = 6
        MODE_TYPE_IDTABLE = 7
        MODE_TYPE = 8
        MODE_TYPE_SENSOR = 9
        MODE_PLACE_IDTABLE = 10
        MODE_PLACE = 11
        MODE_PLACE_TYPE = 12
        MODE_HIERARCHY = 13
        MODE_TAG_IDTABLE = 14
        MODE_ID_STATE = 15
        MODE_ID_SENSOR_STATE = 16
        MODE_ERROR = 999

        if request.tmsdb.tag != '':
            mode = MODE_TAG_IDTABLE
        elif (request.tmsdb.id == 0) and (request.tmsdb.type == '') and (request.tmsdb.sensor == 0) and (request.tmsdb.place == 0) and (request.tmsdb.name == ''):
            mode = MODE_ALL
        elif (request.tmsdb.id == sid) and (request.tmsdb.name != ''):
            mode = MODE_NAME_IDTABLE
        elif (request.tmsdb.id == 0) and (request.tmsdb.name != '') and (request.tmsdb.id != sid) and (request.tmsdb.type == '') and (request.tmsdb.sensor == 0) and (request.tmsdb.place == 0):
            mode = MODE_NAME
        elif (request.tmsdb.id == 0) and (request.tmsdb.name != '') and (request.tmsdb.id != sid) and (request.tmsdb.type == '') and (request.tmsdb.sensor != 0) and (request.tmsdb.place == 0):
            mode = MODE_NAME_SENSOR
        elif (request.tmsdb.id > sid) and (request.tmsdb.type == ''):
            request.tmsdb.id -= sid
            mode = MODE_ID_IDTABLE
        elif (request.tmsdb.id != 0) and (request.tmsdb.id != sid) and (request.tmsdb.type == '') and (request.tmsdb.sensor == 0) and (request.tmsdb.place == 0):
            if (request.tmsdb.state == 1):
                mode = MODE_ID_STATE
            else:
                mode = MODE_ID
        elif (request.tmsdb.id != 0) and (request.tmsdb.id != sid) and (request.tmsdb.type == '') and (request.tmsdb.sensor != 0) and (request.tmsdb.place == 0):
            if (request.tmsdb.state == 1):
                mode = MODE_ID_SENSOR_STATE
            else:
                mode = MODE_ID_SENSOR
        elif (request.tmsdb.id == sid) and (request.tmsdb.type != ''):
            mode = MODE_TYPE_IDTABLE
        elif (request.tmsdb.id == 0) and (request.tmsdb.type != '') and (request.tmsdb.sensor == 0) and (request.tmsdb.place == 0):
            mode = MODE_TYPE
        elif (request.tmsdb.id == 0) and (request.tmsdb.type != '') and (request.tmsdb.sensor != 0) and (request.tmsdb.place == 0):
            mode = MODE_TYPE_SENSOR
        elif (request.tmsdb.id == sid) and (request.tmsdb.type == '') and (request.tmsdb.place != 0):
            mode = MODE_PLACE_IDTABLE
        elif (request.tmsdb.id == 0) and (request.tmsdb.type == '') and (request.tmsdb.place != 0):
            mode = MODE_PLACE
        elif (request.tmsdb.id == 0) and (request.tmsdb.type != '') and (request.tmsdb.place != 0):
            mode = MODE_PLACE_TYPE
        elif (request.tmsdb.id != 0) and (request.tmsdb.type == '') and (request.tmsdb.place == sid):
            mode = MODE_HIERARCHY
        elif (request.tmsdb.id > 0) and ((request.tmsdb.id < 1000) or (request.tmsdb.id > 20002)):
            mode = MODE_ERROR
        else:
            mode = MODE_ERROR

        # Wrong request
        if mode == MODE_ERROR:
            response_tmsdb.note = "Wrong request! Try to check the command!"
            response.tmsdb.append(response_tmsdb)
            return response

        #  Search the ID, type, etc infomation in ID table
        if (mode == MODE_NAME) or (mode == MODE_NAME_SENSOR):
            cursor = db.default.find({'name': request.tmsdb.name})
            temp_type = cursor[0]['type']
            temp_id = cursor[0]['id']
            temp_place = cursor[0]['place']

        # Search the type, name, etc infomation in ID table
        if(mode == MODE_ID) or (mode == MODE_ID_SENSOR) or (mode == MODE_ID_STATE) or (mode == MODE_ID_SENSOR_STATE) or (mode == MODE_HIERARCHY):
            cursor = db.default.find({'id': request.tmsdb.id})
            temp_type = cursor[0]['type']
            temp_name = cursor[0]['name']
            temp_place = cursor[0]['place']

        # Search only using the place tag
        if mode == MODE_PLACE:
            cursor = db.now.find({'place': request.tmsdb.place})
            if cursor.count() == 0:
                response_tmsdb.note = "Wrong request! Try to check the place tag!"
                response.tmsdb.append(response_tmsdb)
                return response
            else:
                for doc in cursor:
                    del doc['_id']
                    response_tmsdb = db_util.document_to_msg(doc, Tmsdb)
                    response.tmsdb.append(response_tmsdb)
                return response

        if mode == MODE_HIERARCHY:
            loop_end_tag = False
            temp_place = request.tmsdb.id

            while temp_place != 0:
                cursor = db.now.find({'id': temp_place}).sort({'time': -1})
                if cursor.count() == 0:
                    response_tmsdb.note = "Wrong request! Try to check the target ID, place info!"
                    response.tmsdb.append(response_tmsdb)
                    return response
                else:
                    for doc in cursor:
                        del doc['_id']
                        response_tmsdb = db_util.document_to_msg(doc, Tmsdb)
                        response.tmsdb.append(response_tmsdb)

                if loop_end_tag:
                    break

                temp_id = response_tmsdb.place  # ?
                cursor = db.default.find({'id': temp_id})
                temp_type = cursor[0]['type']
                temp_name = cursor[0]['name']
                temp_place = cursor[0]['place']

                if temp_id == temp_place:
                    loop_end_tag = True
                else:
                    loop_end_tag = False

                temp_place = temp_id

            return response

        if mode == MODE_ALL:
            cursor = db.default.find()
        elif mode == MODE_TAG_IDTABLE:
            cursor = db.default.find({'tag': {'$regex': request.tmsdb.tag}})
        elif mode == MODE_NAME_IDTABLE:
            cursor = db.default.find({'name': request.tmsdb.name})
        elif mode == MODE_NAME:
            cursor = db.now.find({'name': request.tmsdb.name})
        elif mode == MODE_NAME_SENSOR:
            cursor = db.now.find({'name': request.tmsdb.name, 'sensor': request.tmsdb.sensor})
        elif mode == MODE_ID_IDTABLE:
            cursor = db.default.find({'id': request.tmsdb.id})
        elif mode == MODE_ID:
            cursor = db.now.find({'id': request.tmsdb.id})
        elif mode == MODE_ID_STATE:
            cursor = db.now.find({'id': request.tmsdb.id, 'state': 1})
        elif mode == MODE_ID_SENSOR:
            cursor = db.now.find({'id': request.tmsdb.id, 'sensor': request.tmsdb.sensor})
        elif mode == MODE_ID_SENSOR_STATE:
            cursor = db.now.find({'id': request.tmsdb.id, 'sensor': request.tmsdb.sensor, 'state': 1})
        elif mode == MODE_TYPE_IDTABLE:
            cursor = db.default.find({'type': request.tmsdb.type})
        elif mode == MODE_TYPE:
            cursor = db.now.find({'type': request.tmsdb.type})
        elif mode == MODE_TYPE_SENSOR:
            cursor = db.now.find({'sensor': request.tmsdb.sensor})
        elif mode == MODE_PLACE_IDTABLE:
            cursor = db.default.find({'place': request.tmsdb.place})
        elif mode == MODE_PLACE_TYPE:
            cursor = db.now.find({'place': request.tmsdb.place})

        if cursor.count() == 0:
            response_tmsdb.note = "Wrong request! Try to check the command"
            response.tmsdb.append(response_tmsdb)
            return response
        else:
            for doc in cursor:
                del doc['_id']
                response_tmsdb = db_util.document_to_msg(doc, Tmsdb)
                response.tmsdb.append(response_tmsdb)
            return response


def main(args=None):
    rclpy.init(args=args)
    tms_db_reader = TmsDbReader()
    rclpy.spin(tms_db_reader) 

    tms_db_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
	main()
