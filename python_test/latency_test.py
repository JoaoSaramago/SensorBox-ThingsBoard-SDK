import json
import statistics as stat
import websocket
import threading
import ssl
import timeit
from collections import Counter

from tb_rest_client.rest_client_ce import *
from tb_device_mqtt import TBDeviceMqttClient

DEVICES_COUNT = 50
MSG_PER_DEVICE_PER_SECOND = 1
DURATION = 60

base_url = "thingsboard.rnl.tecnico.ulisboa.pt"
port = ""
server_url = "https://" + base_url
use_tls = True
# base_url = "192.168.1.127"
# port = ":8080"
# server_url = base_url + port
# use_tls = False

username = "tenant@thingsboard.org"
password = "tenant"


elapsed_lst = []
time_lst = []

devices_id = []

failed = 0

rest_client = RestClientCE(base_url=server_url)
rest_client.login(username=username, password=password)
print("Inicio de sessao feito com sucesso!")


class DeviceTester:

    def __init__(self, number, device_id, token):
        self.number = number
        self.device_id = device_id
        self.token = token
        self.start_times = {}
        self.counter = 0
        self.finished = False

        self.client = TBDeviceMqttClient(base_url, self.token)
        self.client.connect()

        rest_token = rest_client.configuration.api_key["X-Authorization"]
        if use_tls:
            ws_url = "wss://" + base_url + "/api/ws/plugins/telemetry?token=" + rest_token
        else:
            ws_url = "ws://" + base_url + ":8080/api/ws/plugins/telemetry?token=" + rest_token
        self.ws = websocket.WebSocketApp(ws_url, on_open=self.on_open, on_message=self.on_message, on_error=self.on_error,
                                    on_close=self.on_close)
        if use_tls:
            self.ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
        else:
            self.ws.run_forever()

    def send_messages(self):
        global failed
        msg_delay = 1/MSG_PER_DEVICE_PER_SECOND
        started = timeit.default_timer()
        while timeit.default_timer() - started < DURATION:
            self.counter += 1
            telemetry = {"count": self.counter}
            self.client.send_telemetry(telemetry)
            self.start_times[self.counter] = timeit.default_timer()
            sleep(msg_delay)
        print(str(self.number) + " - All messages sent!")
        self.finished = True
        started = timeit.default_timer()
        while self.start_times and timeit.default_timer() - started < 8:
            sleep(1)
        failed += len(self.start_times)
        self.ws.close()

    def on_message(self, ws, message):
        end = timeit.default_timer()
        msg = json.loads(message)
        sensors_data = msg["data"]

        if 'count' in sensors_data:
            count = int(sensors_data['count'][0][1])
            if count in list(self.start_times.keys()):
                elapsed = round(end - self.start_times[count], 3)
                self.start_times.pop(count)
                elapsed_lst.append(elapsed)
                time_lst.append(end)

    def on_error(self, ws, error):
        print(error)

    def on_close(self, ws, close_status_code, close_msg):
        # print(str(self.number) + " - WebSocket closed!")
        return

    def on_open(self, ws):
        def run(*args):
            sub_cmd = {
                "tsSubCmds": [{
                    "entityType": "DEVICE",
                    "entityId": device_id.id,
                    "scope": "LATEST_TELEMETRY",
                    "cmdId": 1
                }],
                "historyCmds": [],
                "attrSubCmds": []
            }
            sub_cmd_str = json.dumps(sub_cmd)
            ws.send(sub_cmd_str)
            t_aux = threading.Thread(target=self.send_messages, args=())
            t_aux.start()

        tr = threading.Thread(target=run, args=())
        tr.start()


threads = []

print("Creating", DEVICES_COUNT, "test devices...")
for i in range(DEVICES_COUNT):
    device = Device(name="Tester Latency" + str(i+1))
    device = rest_client.save_device(device)
    device_id = device.id
    devices_id.append(device_id)
    token = rest_client.get_device_credentials_by_device_id(device_id.id).credentials_id

    t = threading.Thread(target=DeviceTester, args=(i+1, device_id, token))
    threads.append(t)

print("Starting Device simulators...")
for thread in threads:
    thread.start()

print("Waiting for each device simulator to send", MSG_PER_DEVICE_PER_SECOND*DURATION, "messages...")
for thread in threads:
    thread.join()

print("Deleting devices...")
for device_id in devices_id:
    rest_client.delete_device(device_id)


mean = stat.mean(elapsed_lst)
print("Mean:", mean)
print("Mediana:", stat.median(elapsed_lst))
print("Desvio Padrao:", round(stat.pstdev(elapsed_lst, mean), 3))
print("Min:", min(elapsed_lst))
print("Max:", max(elapsed_lst))

aux_lst = [round(i) for i in time_lst]
aux_dict = dict(Counter(aux_lst))
mean_rate = stat.mean(aux_dict.values())
median_rate = stat.median(aux_dict.values())

print("Mean msg/s:", round(mean_rate, 1))
print("Median msg/s:", round(median_rate, 1))

print("Failed messages:", failed)
