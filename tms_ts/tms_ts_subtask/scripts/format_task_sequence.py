#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import xml.dom.minidom
import os
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
import re

package_name = 'tms_ts_subtask'


# XML形式で記載されたBT treeを読み込んでstring型に変換し、テキストファイルでの出力及びターミナル上での表示を行うためのノード
# 出力されるstring型の文字列はMongoDBのタスクデータに保存するタスク列として使用する

class ConvertFromXmlToString(Node):
    
    def __init__(self):
        super().__init__('convert_from_xml_to_string')
        
        tms_ts_subtask_package_directory = get_package_share_directory(package_name)

        self.declare_parameter('bt_tree_xml_file_name', 'sample_construction_tree')
        self.declare_parameter("output_text_file_directory_path", tms_ts_subtask_package_directory + '/config')
        self.declare_parameter("output_file_name", 'task_sequence')
        self.declare_parameter("output_txt_file", False)

        xml_file_name = self.get_parameter("bt_tree_xml_file_name").get_parameter_value().string_value
        output_text_file_directory = self.get_parameter("output_text_file_directory_path").get_parameter_value().string_value
        output_file_name = self.get_parameter("output_file_name").get_parameter_value().string_value
        output_txt_file = self.get_parameter("output_txt_file").get_parameter_value().bool_value

        output_file_path = output_text_file_directory + "/" + output_file_name + '.txt'
        xml_file_path = tms_ts_subtask_package_directory + '/config/' + xml_file_name + '.xml'

        tree = ET.parse(xml_file_path)
        root = tree.getroot()
        xml_string = ET.tostring(root, encoding='unicode', method='xml')
        xml_string = '\n'.join(line.strip() for line in xml_string.split('\n'))
        xml_string = re.sub(r'\n', '', xml_string)
        # xml_string = ''.join(xml_string.split())
        self.get_logger().info("task_sequence:  " + xml_string)
        if output_txt_file == True:
            self.create_text_file(output_file_path, xml_string)
        
    def create_text_file(self, file_path, content):
        try:
            with open(file_path, 'w') as file:
                file.write(content)
            print(f"File '{file_path}' created successfully.")
        except Exception as e:
            print(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    convert_from_xml_to_string = ConvertFromXmlToString()
    rclpy.spin(convert_from_xml_to_string)
    convert_from_xml_to_string.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()