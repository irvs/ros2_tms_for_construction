import subprocess
from pymongo import MongoClient
import gridfs
import os
import csv
from datetime import datetime
import laspy
import argparse


parser = argparse.ArgumentParser(description="LASファイルから画像を生成しMongoDBに保存する")
parser.add_argument("las_file", help="入力LASファイルのファイル名（例: CollageWeb.las）")
parser.add_argument("--output", help="出力画像のファイル名（例: output.png）", default="outputTest.png")
args = parser.parse_args()

#input_file_name = args.las_file
input_file_name = os.path.basename(args.las_file)

# MongoDB に接続
client = MongoClient('mongodb://localhost:27017/')  # MongoDB がローカルにある場合
#db = client['heightmap_db']  # データベース名
db = client['rostmsdb']  # データベース名
fs = gridfs.GridFS(db)  # GridFS の初期化

# 入力ファイルと出力ファイルのパス
#input_file_name = 'CollageWeb.las'
input_file = '/data/' + input_file_name
output_heightmap_name =  'outputTest.png'
output_heightmap = '/data/' + output_heightmap_name
mongodb_heightmap_name =  'outputTest.png'
output_texture_name = 'outputTest1.png'
output_texture = '/data/' + output_texture_name
mongodb_texture_name =  'outputTest.png'
width = 2048
height = 2048

csv_file = '/data/elevation_min_max.csv'
# 現在の作業ディレクトリを取得
current_dir = os.getcwd()

####
# LASファイルを読み込む
las = laspy.read(input_file_name)  # 'path_to_your_file.las' を実際のファイルパスに置き換えてください
# X, Y, Zの座標を取得
x = las.x
y = las.y
z = las.z
# 最大値と最小値を計算
x_min, x_max = x.min(), x.max()
y_min, y_max = y.min(), y.max()
z_min, z_max = z.min(), z.max()
# 結果を表示
print(f"X座標の最小値: {x_min}, 最大値: {x_max}")
print(f"Y座標の最小値: {y_min}, 最大値: {y_max}")
print(f"Z座標の最小値: {z_min}, 最大値: {z_max}")
####

# 1. las2heightmap を実行して画像を生成
command = [
    "sudo","docker", "run", "--name", "las2heightmap_container", "--rm",  # コンテナ名を指定
    "-v", f"{current_dir}:/data",  # 現在のディレクトリを絶対パスで指定
    "las2heightmap", "-i", input_file, "-o", output_heightmap, 
    "-W", str(width), "-H", str(height),
    "-elevation_csv", csv_file, 
    "-min_x", str(x_min), "-max_x", str(x_max), "-min_y", str(y_min), "-max_y", str(y_max), "-min_z", str(z_min),
]

# subprocess でコマンドを実行
try:
    subprocess.run(command, check=True)
    print(f"las2heightmap executed successfully. Output saved to {output_heightmap}")
except subprocess.CalledProcessError as e:
    print(f"Error occurred while running las2heightmap: {e}")
    exit(1)


# 4. chmod を実行してパーミッションを変更
chmod_command = [
    "sudo", "docker", "run", "--rm",
    "-v", f"{current_dir}:/data",  # 現在のディレクトリを絶対パスで指定
    "busybox", "chmod", "755", "/data/outputTest.png"  # busyboxを使ってchmodを実行
]

# subprocess で chmod を実行
try:
    subprocess.run(chmod_command, check=True)
    print(f"Permissions of {output_heightmap} changed successfully.")
except subprocess.CalledProcessError as e:
    print(f"Error occurred while changing permissions: {e}")
    exit(1)

##################
# CSVファイルのパス
csv_file_path = os.path.join(current_dir, 'elevation_min_max.csv')

# CSVファイルを開いて読み込む
def read_csv(csv_file_path):
    with open(csv_file_path, mode='r') as file:
        csv_reader = csv.reader(file)
        headers = next(csv_reader)  # ヘッダー行をスキップ
        for row in csv_reader:
            # 行ごとにデータを処理
            min_x, max_x, min_y, max_y, min_z, max_z = row
            print(f"minX: {min_x}, maxX: {max_x}, minY: {min_y}, maxY: {max_y}, minZ: {min_z}, maxZ: {max_z}")
            return min_x, max_x, min_y, max_y, min_z, max_z  # 必要な値を返す

# CSVファイルを読み込む
min_x, max_x, min_y, max_y, min_z, max_z = read_csv(csv_file_path)
######################
upload_time = datetime.now()
StDytype = 'heightmap'
DataId = 4031
DataHeight = float(x_max) - float(x_min)
DataWidth = float(y_max) - float(y_min)
DataElevation = float(z_max) - float(z_min)
DataKinds = 'heightmap'

# 画像ファイルを MongoDB に保存
try:
    with open(output_heightmap_name, 'rb') as f:
        image_data = f.read()  # 画像データを一度読み取る
        # 画像データを GridFS に保存
        fs.put(image_data, filename=args.output, time=upload_time , type=StDytype, id=DataId, height=DataHeight, width=DataWidth, elevation=DataElevation, offset_x=0, offset_y=0, DataType=DataKinds)

    print(f"画像ファイルは MongoDB に保存されました。")
except Exception as e:
    print(f"Error occurred while saving image to MongoDB: {e}")
    exit(1)

######################
######################
# 1. texture を実行して画像を生成
command_2 = [
    "sudo", "docker", "run", "--name", "las2heightmap_container", "--rm",  # コンテナ名を指定
    "-v", f"{current_dir}:/data",  # 現在のディレクトリを絶対パスで指定
    "las2heightmap", "-i", input_file, "-o", output_texture, 
    "-W", str(width), "-H", str(height),
    "-elevation_csv", csv_file,  # 標高データをCSVとして出力
    "-rgb", "true",
    "-min_x", str(x_min), "-max_x", str(x_max), "-min_y", str(y_min), "-max_y", str(y_max), "-min_z", str(z_min),
]

# subprocess でコマンドを実行
try:
    subprocess.run(command_2, check=True)
    print(f"las2heightmap executed successfully. Output saved to {output_texture}")
except subprocess.CalledProcessError as e:
    print(f"Error occurred while running las2heightmap: {e}")
    exit(1)

# 4. chmod を実行してパーミッションを変更
chmod_command_2 = [
    "sudo", "docker", "run", "--rm",
    "-v", f"{current_dir}:/data",  # 現在のディレクトリを絶対パスで指定
    "busybox", "chmod", "755", "/data/outputTest.png"  # busyboxを使ってchmodを実行
]

# subprocess で chmod を実行
try:
    subprocess.run(chmod_command_2, check=True)
    print(f"Permissions of {output_texture} changed successfully.")
except subprocess.CalledProcessError as e:
    print(f"Error occurred while changing permissions: {e}")
    exit(1)

output_texture_name = os.path.join(current_dir, output_texture_name)


######################
# 現在の時刻を取得
upload_time = datetime.now()
StDytype = 'texture'
DataId = 4031
DataKinds = 'texture'


# 5. 画像ファイルを MongoDB に保存
try:
    with open(output_texture_name, 'rb') as f:
        image_data = f.read()

        # 画像データを GridFS に保存
        fs.put(image_data, filename=args.output, time=upload_time , type=StDytype, id=DataId, height=DataHeight, width=DataWidth, elevation=DataElevation, offset_x = 00, offset_y = 00, DataType=DataKinds)

    print(f"画像ファイルは MongoDB に保存されました。")
except Exception as e:
    print(f"Error occurred while saving image to MongoDB: {e}")
    exit(1)