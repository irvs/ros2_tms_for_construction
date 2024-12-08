
本ブランチ�?�、[ros2-tms for constructionのmainブランチ](https://github.com/irvs/ros2_tms_for_construction)にあがって�?る�?�ログラ�?をdocker上でも実行可能にしたも�?�です。一部コマンドがmainブランチで説明したものと異なる�?�で�?"環�?構�?"と"Demo"の部�?のみ、こちら�?�ReadMeにて説明します�?

使用方法�?�以下�?�とおりです�?

## 環�?構�?

### コン�?ナ�?�作�??
以下�?�手�??でDockerFileからDockerイメージをビルドしてください�?
```
cd && git clone -b develop/docker-model  https://github.com/irvs/ros2_tms_for_construction.git
cd ros2_tms_for_construction
sudo docker build --no-cache -t ros2_tms_for_construction -f DockerFile .
```

### コン�?ナ�?�実行と表示
```
sudo docker run --privileged -p 6080:80 -p 10000:10000 --shm-size=512m -v /etc/localtime:/etc/localtime:ro -v /etc/timezone:/etc/timezone:ro -it ros2_tms_for_construction
```
上記コマンドを実行した状態で、ブラウザを開�?て http://127.0.0.1:6080/ にアクセスしてください。すると以下�?�画面が表示される�?�で"接�?"をクリ�?クしてください�?

![](docs/docker_1.png)

すると以下�?�ようにコン�?ナ�??で作業することができます�?

![](docs/docker_2.png)

## Demo

 ここでは、mainブランチで紹介されて�?るデモの�?�?4 ~ 7 につ�?てご紹介します�? 詳細は以下�?�リンク先に記載�?�とおりです。実行に際し、コン�?ナ�??で開いた各�?の端末上では最初に以下�?�コマンドを実行してください�?
 ```
 cd ~/ros2-tms-for-construction_ws
 source install/setup.bash
 ```

 4. [Try running the task schedular](CHAPTER4.md)
 5. [Try running the task schedular with OperaSim-PhysX](CHAPTER5.md)
 6. [Insert new task data to tms_db](CHAPTER6.md)
 7. [How to update parameters in mongodb based on topics from sensing pc.](CHAPTER7.md)

## Acknowledgement

This Dockerfile is based on [Tiryoh/docker-ros2-desktop-vnc](https://github.com/Tiryoh/docker-ros2-desktop-vnc), licensed under the Apache License 2.0.








