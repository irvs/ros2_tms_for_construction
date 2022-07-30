import gridfs
from rclpy.node import Node
from tms_msg_db.srv import TmsdbGridFSGetData
import rclpy
import tms_db_manager.tms_db_util as db_util


class TmsDbReaderGridFS(Node):
    """Read file data from MongoDB."""
  
    def __init__(self):
        super().__init__('tms_db_reader_gridfs')

        # declare parameter
        self.declare_parameter('db_host', 'localhost')
        self.declare_parameter('db_port', 27017)

        # get parameter
        self.db_host = self.get_parameter('db_host').get_parameter_value().string_value
        self.db_port = self.get_parameter('db_port').get_parameter_value().integer_value

        self.db  = db_util.connect_db('rostmsdb', self.db_host, self.db_port)
        self.srv = self.create_service(TmsdbGridFSGetData, 'tms_db_reader_gridfs', self.db_reader_srv_callback)


    def db_reader_srv_callback(self, request, response):
        """
        Respond to requests from client nodes.

        Parameters
        ----------
        request
            Request from client node.
        response
            Response to client node.

        Returns
        -------
        response
            Response to client node.
        """
        fs = gridfs.GridFS(self.db)
        file_obj = fs.find_one({'filename': request.filename})
        if file_obj is None:
            response.result = False
            return response

        f = open(request.filename, 'wb')
        f.write(file_obj.read())
        f.close()

        response.result = True
        return response


def main(args=None):
    rclpy.init(args=args)
    tms_db_reader_gridfs = TmsDbReaderGridFS()
    rclpy.spin(tms_db_reader_gridfs) 

    tms_db_reader_gridfs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
	main()
