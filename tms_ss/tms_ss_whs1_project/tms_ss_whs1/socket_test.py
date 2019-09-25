# クライアントを作成

import socket

def split_n(text, n):
    return [ text[i*n:i*n+n] for i in range(int(len(text)/n)) ]

while True:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # サーバを指定
        s.connect(('192.168.4.92', 801))
        # サーバにメッセージを送る
        # s.sendall(b'hello')
        # ネットワークのバッファサイズは1024。サーバからの文字列を取得する
        data = s.recv(1024)
        #
        print(split_n(data.hex(), 8))