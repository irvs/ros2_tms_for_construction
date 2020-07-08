#rostmswlan928
import requests
import json
import time
import sys

url = "http://192.168.11.2/api/gHPwUPfOwU9tTZJcCYi-nIrCoe1SYGvKXK1Urst5/lights/1/state"

def main():
    global url
    #url = 'http://192.168.11.2/api/-nkSfDvTobVXyA6AXF8uWQMY-oDepcqVCF7ubQvv/lights/1/state'
    # "on": true/false, "bri": 1-254, "hue": 0:65535. "sat": 0:254, "xy": [0~1, 0~1]
    # data = '{"on": true, "bri": 254, "hue": 10, "sat": 100}'
    data = '{"on": true, "xy": [0.7, 0.3]}'

    headers = {"Content-Type": "application/json"}

    response = requests.put(url, data=data, headers=headers)
    res = response.json()

    print(res)
    #print(response.status_code)    # HTTPのステータスコード取得
    #print(response.text)    # レスポンスのHTMLを文字列で取得

    
def rainbow():
    global url
    #url = 'http://192.168.11.2/api/-nkSfDvTobVXyA6AXF8uWQMY-oDepcqVCF7ubQvv/lights/1/state'

    # "on": true/false, "bri": 1-254, "hue": 0:65535. "sat": 0:254, "xy": [0~1, 0~1]
    # data = '{"on": true, "bri": 254, "hue": 10, "sat": 100}'
    # data = '{"on": true, "xy": [0.7, 0.3]}'

    headers = {"Content-Type": "application/json"}

    try:
        while True:
            for i in range(0, 65536, 1000):
                #data = '{"on": true, "bri": 254, "hue": 10, "sat": 100}'
                data = '{"on": true, "bri": 254, "hue": ' + str(i) + ', "sat": 100}'
                response = requests.put(url, data=data, headers=headers)
                #res = response.json()
                #print(res)
                time.sleep(0.1)

    except KeyboardInterrupt:
        sys.exit(0)

if __name__ == '__main__':
    #main()
    rainbow()
