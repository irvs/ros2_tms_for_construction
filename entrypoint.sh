#!/bin/bash

# This script is based on docker-ros2-desktop-vnc/humble/entrypoint.sh(GIthub: https://github.com/Tiryoh/docker-ros2-desktop-vnc/blob/master/humble/entrypoint.sh) licensed under the Apache License 2.0.

# ローカルホストに対するアクセスを許可
xhost +local:root

# MongoDBサーバ�?�をバ�?クグラウンドで起�?
mongod --fork --logpath /var/log/mongodb/mongod.log --dbpath /var/lib/mongodb

cd /home/ubuntu/ros2-tms-for-construction_ws/src/ros2_tms_for_construction/demo
unzip rostmsdb_collections.zip
mongorestore dump
cd 

source /opt/ros/humble/setup.bash
cd /home/ubuntu/ros2-tms-for-construction_ws
colcon build --packages-select com3_msgs tms_msg_ur && source install/setup.bash
colcon build && source install/setup.bash

# Create User
USER=${USER:-root}
HOME=/root
if [ "$USER" != "root" ]; then
    echo "* enable custom user: $USER"
    useradd --create-home --shell /bin/bash --user-group --groups adm,sudo $USER
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
    if [ -z "$PASSWORD" ]; then
        echo "  set default password to \"ubuntu\""
        PASSWORD=ubuntu
    fi
    HOME=/home/$USER
    echo "$USER:$PASSWORD" | /usr/sbin/chpasswd 2> /dev/null || echo ""
    cp -r /root/{.config,.gtkrc-2.0,.asoundrc} ${HOME} 2>/dev/null
    chown -R $USER:$USER ${HOME}
    [ -d "/dev/snd" ] && chgrp -R adm /dev/snd
fi

# VNC password
VNC_PASSWORD=${PASSWORD:-ubuntu}

mkdir -p $HOME/.vnc
echo $VNC_PASSWORD | vncpasswd -f > $HOME/.vnc/passwd
chmod 600 $HOME/.vnc/passwd
chown -R $USER:$USER $HOME
sed -i "s/password = WebUtil.getConfigVar('password');/password = '$VNC_PASSWORD'/" /usr/lib/novnc/app/ui.js

# xstartup
XSTARTUP_PATH=$HOME/.vnc/xstartup
cat << EOF > $XSTARTUP_PATH
#!/bin/sh
unset DBUS_SESSION_BUS_ADDRESS
mate-session
EOF
chown $USER:$USER $XSTARTUP_PATH
chmod 755 $XSTARTUP_PATH

# vncserver launch
VNCRUN_PATH=$HOME/.vnc/vnc_run.sh
cat << EOF > $VNCRUN_PATH
#!/bin/sh

if [ $(uname -m) = "aarch64" ]; then
    LD_PRELOAD=/lib/aarch64-linux-gnu/libgcc_s.so.1 vncserver :1 -fg -geometry 1920x1080 -depth 24
else
    vncserver :1 -fg -geometry 1920x1080 -depth 24
fi
EOF

# Supervisor
CONF_PATH=/etc/supervisor/conf.d/supervisord.conf
cat << EOF > $CONF_PATH
[supervisord]
nodaemon=true
user=root
[program:vnc]
command=gosu '$USER' bash '$VNCRUN_PATH'
[program:novnc]
command=gosu '$USER' bash -c "websockify --web=/usr/lib/novnc 80 localhost:5901"
EOF

# colcon
BASHRC_PATH=$HOME/.bashrc
grep -F "source /opt/ros/$ROS_DISTRO/setup.bash" $BASHRC_PATH || echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $BASHRC_PATH
grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" $BASHRC_PATH || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> $BASHRC_PATH
chown $USER:$USER $BASHRC_PATH

# Fix rosdep permission
mkdir -p $HOME/.ros
cp -r /root/.ros/rosdep $HOME/.ros/rosdep
chown -R $USER:$USER $HOME/.ros

# Add terminator shortcut
mkdir -p $HOME/Desktop
cat << EOF > $HOME/Desktop/terminator.desktop
[Desktop Entry]
Name=Terminator
Comment=Multiple terminals in one window
TryExec=terminator
Exec=terminator
Icon=terminator
Type=Application
Categories=GNOME;GTK;Utility;TerminalEmulator;System;
StartupNotify=true
X-Ubuntu-Gettext-Domain=terminator
X-Ayatana-Desktop-Shortcuts=NewWindow;
Keywords=terminal;shell;prompt;command;commandline;
[NewWindow Shortcut Group]
Name=Open a New Window
Exec=terminator
TargetEnvironment=Unity
EOF
cat << EOF > $HOME/Desktop/firefox.desktop
[Desktop Entry]
Version=1.0
Name=Firefox Web Browser
Name[ar]=�?تص�?ح ا�?ويب �?َيَر�?ُ�?ْس
Name[ast]=Restolador web Firefox
Name[bn]=ফায়ারফক্স ওয়�?ব ব্রাউজার
Name[ca]=Navegador web Firefox
Name[cs]=Firefox Webový prohlíže�?
Name[da]=Firefox - internetbrowser
Name[el]=�?εριηγη�?ή�? Firefox
Name[es]=Navegador web Firefox
Name[et]=Firefoxi veebibrauser
Name[fa]=�?رورگر ای�?تر�?ت�? Firefox
Name[fi]=Firefox-selain
Name[fr]=Navigateur Web Firefox
Name[gl]=Navegador web Firefox
Name[he]=דפדפ�? האי�?טר�?�? Firefox
Name[hr]=Firefox web preglednik
Name[hu]=Firefox webböngész�?
Name[it]=Firefox Browser Web
Name[ja]=Firefox ウェブ�?�ブラウザ
Name[ko]=Firefox ?�� ?��라?��??�
Name[ku]=Geroka torê Firefox
Name[lt]=Firefox interneto naršykl�?
Name[nb]=Firefox Nettleser
Name[nl]=Firefox webbrowser
Name[nn]=Firefox Nettlesar
Name[no]=Firefox Nettleser
Name[pl]=Przegl�?darka WWW Firefox
Name[pt]=Firefox Navegador Web
Name[pt_BR]=Navegador Web Firefox
Name[ro]=Firefox �? Navigator Internet
Name[ru]=Веб-браузер Firefox
Name[sk]=Firefox - internetový prehliada�?
Name[sl]=Firefox spletni brskalnik
Name[sv]=Firefox webbläsare
Name[tr]=Firefox Web Tarayıcısı
Name[ug]=Firefox تور�?�?رگ�?
Name[uk]=Веб-браузер Firefox
Name[vi]=Trình duyệt web Firefox
Name[zh_CN]=Firefox 网络浏览器
Name[zh_TW]=Firefox 網路瀏覽器
Comment=Browse the World Wide Web
Comment[ar]=تص�?ح ا�?شب�?ة ا�?ع�?�?بوتية ا�?عا�?�?ية
Comment[ast]=Restola pela Rede
Comment[bn]=�?ন্টারন�?�? ব্রাউ�? করুন
Comment[ca]=Navegueu per la web
Comment[cs]=Prohlížení stránek World Wide Webu
Comment[da]=Surf på internettet
Comment[de]=Im Internet surfen
Comment[el]=Μπορεί�?ε να περιηγηθεί�?ε σ�?ο διαδίκ�?�?ο (Web)
Comment[es]=Navegue por la web
Comment[et]=Lehitse veebi
Comment[fa]=ص�?حات شبک�? ج�?ا�?�? ای�?تر�?ت را �?رور �?�?ایید
Comment[fi]=Selaa Internetin WWW-sivuja
Comment[fr]=Naviguer sur le Web
Comment[gl]=Navegar pola rede
Comment[he]=גליש�? ברחב�? האי�?טר�?�?
Comment[hr]=Pretražite web
Comment[hu]=A világháló böngészése
Comment[it]=Esplora il web
Comment[ja]=ウェブを閲覧しま�?
Comment[ko]=?��?�� ?��?�� ?��?��?��?��
Comment[ku]=Li torê bigere
Comment[lt]=Naršykite internete
Comment[nb]=Surf på nettet
Comment[nl]=Verken het internet
Comment[nn]=Surf på nettet
Comment[no]=Surf på nettet
Comment[pl]=Przegl�?danie stron WWW 
Comment[pt]=Navegue na Internet
Comment[pt_BR]=Navegue na Internet
Comment[ro]=Navigați pe Internet
Comment[ru]=До�?туп в Интерне�?
Comment[sk]=Prehliadanie internetu
Comment[sl]=Brskajte po spletu
Comment[sv]=Surfa på webben
Comment[tr]=İnternet'te Gezinin
Comment[ug]=د�?�?يادى�?�? توربەت�?ەر�?�? �?�?رگى�?�? بو�?ىد�?
Comment[uk]=Перегляд �?торінок �?нтернет�?
Comment[vi]=Đ�? duyệt các trang web
Comment[zh_CN]=浏览互联�?
Comment[zh_TW]=瀏覽網際網路
GenericName=Web Browser
GenericName[ar]=�?تص�?ح ويب
GenericName[ast]=Restolador Web
GenericName[bn]=ওয়�?ব ব্রাউজার
GenericName[ca]=Navegador web
GenericName[cs]=Webový prohlíže�?
GenericName[da]=Webbrowser
GenericName[el]=�?εριηγη�?ή�? διαδικ�?ύο�?
GenericName[es]=Navegador web
GenericName[et]=Veebibrauser
GenericName[fa]=�?رورگر ای�?تر�?ت�?
GenericName[fi]=WWW-selain
GenericName[fr]=Navigateur Web
GenericName[gl]=Navegador Web
GenericName[he]=דפדפ�? אי�?טר�?�?
GenericName[hr]=Web preglednik
GenericName[hu]=Webböngész�?
GenericName[it]=Browser web
GenericName[ja]=ウェブ�?�ブラウザ
GenericName[ko]=?�� ?��라?��??�
GenericName[ku]=Geroka torê
GenericName[lt]=Interneto naršykl�?
GenericName[nb]=Nettleser
GenericName[nl]=Webbrowser
GenericName[nn]=Nettlesar
GenericName[no]=Nettleser
GenericName[pl]=Przegl�?darka WWW
GenericName[pt]=Navegador Web
GenericName[pt_BR]=Navegador Web
GenericName[ro]=Navigator Internet
GenericName[ru]=Веб-браузер
GenericName[sk]=Internetový prehliada�?
GenericName[sl]=Spletni brskalnik
GenericName[sv]=Webbläsare
GenericName[tr]=Web Tarayıcı
GenericName[ug]=تور�?�?رگ�?
GenericName[uk]=Веб-браузер
GenericName[vi]=Trình duyệt Web
GenericName[zh_CN]=网络浏览器
GenericName[zh_TW]=網路瀏覽器
Keywords=Internet;WWW;Browser;Web;Explorer
Keywords[ar]=ا�?تر�?ت;إ�?تر�?ت;�?تص�?ح;ويب;وب
Keywords[ast]=Internet;WWW;Restolador;Web;Esplorador
Keywords[ca]=Internet;WWW;Navegador;Web;Explorador;Explorer
Keywords[cs]=Internet;WWW;Prohlíže�?;Web;Explorer
Keywords[da]=Internet;Internettet;WWW;Browser;Browse;Web;Surf;Nettet
Keywords[de]=Internet;WWW;Browser;Web;Explorer;Webseite;Site;surfen;online;browsen
Keywords[el]=Internet;WWW;Browser;Web;Explorer;Διαδίκ�?�?ο;�?εριηγη�?ή�?;Firefox;Φιρε�?ο�?;Ιν�?ερνε�?
Keywords[es]=Explorador;Internet;WWW
Keywords[fi]=Internet;WWW;Browser;Web;Explorer;selain;Internet-selain;internetselain;verkkoselain;netti;surffaa
Keywords[fr]=Internet;WWW;Browser;Web;Explorer;Fureteur;Surfer;Navigateur
Keywords[he]=דפדפ�?;אי�?טר�?�?;רשת;אתרי�?;אתר;פיירפוקס;מוזיל�?;
Keywords[hr]=Internet;WWW;preglednik;Web
Keywords[hu]=Internet;WWW;Böngész�?;Web;Háló;Net;Explorer
Keywords[it]=Internet;WWW;Browser;Web;Navigatore
Keywords[is]=Internet;WWW;Vafri;Vefur;Netvafri;Flakk
Keywords[ja]=Internet;WWW;Web;インターネッ�?;ブラウザ;ウェ�?;エクスプローラ
Keywords[nb]=Internett;WWW;Nettleser;Explorer;Web;Browser;Nettside
Keywords[nl]=Internet;WWW;Browser;Web;Explorer;Verkenner;Website;Surfen;Online 
Keywords[pt]=Internet;WWW;Browser;Web;Explorador;Navegador
Keywords[pt_BR]=Internet;WWW;Browser;Web;Explorador;Navegador
Keywords[ru]=Internet;WWW;Browser;Web;Explorer;интерне�?;браузер;веб;�?айр�?ок�?;огнели�?
Keywords[sk]=Internet;WWW;Prehliada�?;Web;Explorer
Keywords[sl]=Internet;WWW;Browser;Web;Explorer;Brskalnik;Splet
Keywords[tr]=İnternet;WWW;Tarayıcı;Web;Gezgin;Web sitesi;Site;sörf;çevrimiçi;tara
Keywords[uk]=Internet;WWW;Browser;Web;Explorer;�?нтерне�?;мережа;перегляда�?;огляда�?;браузер;веб;�?айр�?ок�?;вогнели�?;перегляд
Keywords[vi]=Internet;WWW;Browser;Web;Explorer;Trình duyệt;Trang web
Keywords[zh_CN]=Internet;WWW;Browser;Web;Explorer;网页;浏�?;上�?;火�?;Firefox;ff;互联�?;网�?;
Keywords[zh_TW]=Internet;WWW;Browser;Web;Explorer;網際網路;網路;瀏覽器;上網;網�?;火�?
Exec=firefox %u
Terminal=false
X-MultipleArgs=false
Type=Application
Icon=firefox
Categories=GNOME;GTK;Network;WebBrowser;
MimeType=text/html;text/xml;application/xhtml+xml;application/xml;application/rss+xml;application/rdf+xml;image/gif;image/jpeg;image/png;x-scheme-handler/http;x-scheme-handler/https;x-scheme-handler/ftp;x-scheme-handler/chrome;video/webm;application/x-xpinstall;
StartupNotify=true
Actions=new-window;new-private-window;

[Desktop Action new-window]
Name=Open a New Window
Name[ar]=ا�?تح �?ا�?ذة جديدة
Name[ast]=Abrir una ventana nueva
Name[bn]=Abrir una ventana nueva
Name[ca]=Obre una finestra nova
Name[cs]=Otevřít nové okno
Name[da]=�?bn et nyt vindue
Name[de]=Ein neues Fenster öffnen
Name[el]=Νέο παράθ�?ρο
Name[es]=Abrir una ventana nueva
Name[fi]=Avaa uusi ikkuna
Name[fr]=Ouvrir une nouvelle fenêtre
Name[gl]=Abrir unha nova xanela
Name[he]=פתיחת חלו�? חדש
Name[hr]=Otvori novi prozor
Name[hu]=Új ablak nyitása
Name[it]=Apri una nuova finestra
Name[ja]=新しいウィンドウを開�?
Name[ko]=?�� ?�� ?��?��
Name[ku]=Paceyeke nû veke
Name[lt]=Atverti nauj�? lang�?
Name[nb]=�?pne et nytt vindu
Name[nl]=Nieuw venster openen
Name[pt]=Abrir nova janela
Name[pt_BR]=Abrir nova janela
Name[ro]=Deschide o fereastr�? nou�?
Name[ru]=Новое окно
Name[sk]=Otvoriť nové okno
Name[sl]=Odpri novo okno
Name[sv]=Öppna ett nytt fönster
Name[tr]=Yeni pencere aç 
Name[ug]=يېڭ�? �?�?ز�?ە�? ئې�?ىش
Name[uk]=Відкрити нове вікно
Name[vi]=M�? cửa s�? mới
Name[zh_CN]=新建窗口
Name[zh_TW]=開啟新視�?
Exec=firefox -new-window

[Desktop Action new-private-window]
Name=Open a New Private Window
Name[ar]=ا�?تح �?ا�?ذة جديدة �?�?تص�?ح ا�?خاص
Name[ca]=Obre una finestra nova en mode d'incògnit
Name[cs]=Otevřít nové anonymní okno
Name[de]=Ein neues privates Fenster öffnen
Name[el]=Νέο ιδιω�?ικ�? παράθ�?ρο
Name[es]=Abrir una ventana privada nueva
Name[fi]=Avaa uusi yksityinen ikkuna
Name[fr]=Ouvrir une nouvelle fenêtre de navigation privée
Name[he]=פתיחת חלו�? גליש�? פרטית חדש
Name[hu]=Új privát ablak nyitása
Name[it]=Apri una nuova finestra anonima
Name[nb]=�?pne et nytt privat vindu
Name[ru]=Новое приватное окно
Name[sl]=Odpri novo okno zasebnega brskanja
Name[sv]=Öppna ett nytt privat fönster
Name[tr]=Yeni gizli pencere aç
Name[uk]=Відкрити нове вікно �? потайливом�? режим�?
Name[zh_TW]=開啟新隱私瀏覽視�?
Exec=firefox -private-window
EOF
cat << EOF > $HOME/Desktop/codium.desktop
[Desktop Entry]
Name=VSCodium
Comment=Code Editing. Redefined.
GenericName=Text Editor
Exec=/usr/share/codium/codium --unity-launch %F
Icon=vscodium
Type=Application
StartupNotify=false
StartupWMClass=VSCodium
Categories=TextEditor;Development;IDE;
MimeType=text/plain;inode/directory;application/x-codium-workspace;
Actions=new-empty-window;
Keywords=vscode;

[Desktop Action new-empty-window]
Name=New Empty Window
Exec=/usr/share/codium/codium --new-window %F
Icon=vscodium
EOF
chown -R $USER:$USER $HOME/Desktop

# clearup
PASSWORD=
VNC_PASSWORD=

echo "============================================================================================"
echo "NOTE 1: --security-opt seccomp=unconfined flag is required to launch Ubuntu Jammy based image."
echo -e 'See \e]8;;https://github.com/Tiryoh/docker-ros2-desktop-vnc/pull/56\e\\https://github.com/Tiryoh/docker-ros2-desktop-vnc/pull/56\e]8;;\e\\'
echo "NOTE 2: Before stopping to commit docker container to new docker image, log out first."
echo -e 'See \e]8;;https://github.com/Tiryoh/docker-ros2-desktop-vnc/issue/131\e\\https://github.com/Tiryoh/docker-ros2-desktop-vnc/issue/131\e]8;;\e\\'
echo "============================================================================================"

exec /bin/tini -- supervisord -n -c /etc/supervisor/supervisord.conf

