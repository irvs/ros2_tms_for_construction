# This file is used to setup the environment for ROS2-TMS for Construction.

##setup.shの作成手順################################################################################

# この下からros2-tms for constructionのreadme資料を参考に環境構築用のスクリプトを作成して下さい
# ros2の環境構築の一部のみ参考に示しておきます
# このファイルの実行コマンドは以下のとおり

# cd ~/[woekspace]/src/ros2_tms_for_construction
# sudo chmod +x setup.sh
# sudo ./setup.sh

# 上記コマンドを実行すると、以下のコマンドをターミナル上で打ち込んだ際と同様な出力が得られます

# locale  # check for UTF-8

# sudo apt update && sudo apt install locales
# sudo locale-gen en_US en_US.UTF-8
# sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# export LANG=en_US.UTF-8

# locale  # verify settings
##################################################################################################

# Setup ROS2
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings





