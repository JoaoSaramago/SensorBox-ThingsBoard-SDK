import json
import websocket
import _thread
import ssl
# Importing models and REST client class from Community Edition version
from tb_rest_client.rest_client_ce import *
from tb_rest_client.rest import ApiException

# ThingsBoard server url
base_url = "thingsboard.rnl.tecnico.ulisboa.pt"
port = ""

if port:
    server_url = "https://" + base_url + ":" + port
else:
    server_url = "https://" + base_url

username = "tenant@thingsboard.org"
password = "tenant"

actuators_deviceID_str = "5622e270-47cc-11ec-95a5-f79b1c49d872"
sensors_deviceID_str = "8ce39bb0-47cc-11ec-95a5-f79b1c49d872"


# Creating the REST client object with context manager to get auto token refresh
with RestClientCE(base_url=server_url) as rest_client:
    actuators_deviceID = DeviceId('DEVICE', actuators_deviceID_str)

    def on_message(ws, message):
        msg = json.loads(message)
        sensors_data = msg["data"]

        if not sensors_data:
            print(message)
            return

        if 'temperature' in sensors_data:
            temp = sensors_data["temperature"][0][1]
            print("Temperature:", temp)
            temp_alarm = True if float(temp) > 17.0 else False
            body_alarm = {
                "method": "setLedTempAlarm",
                "params": {
                    "value": temp_alarm
                }
            }
            body_alarm_str = json.dumps(body_alarm)
            _thread.start_new_thread(rest_client.handle_two_way_device_rpc_request, (body_alarm_str, actuators_deviceID))

        if 'blink_interval' in sensors_data:
            interval = sensors_data["blink_interval"][0][1]
            print("Blink Interval:", interval)
            body_blink = {
                "method": "setLedBlinkInterval",
                "params": {
                    "value": interval
                }
            }
            body_blink_str = json.dumps(body_blink)
            _thread.start_new_thread(rest_client.handle_two_way_device_rpc_request, (body_blink_str, actuators_deviceID))

        if 'light_intensity' in sensors_data:
            intensity = sensors_data["light_intensity"][0][1]
            intensity = 255 - int(intensity)
            print("Light Intensity:", intensity)
            body_intensity = {
                "method": "setLedIntensity",
                "params": {
                    "value": intensity
                }
            }
            body_intensity_str = json.dumps(body_intensity)
            _thread.start_new_thread(rest_client.handle_two_way_device_rpc_request, (body_intensity_str, actuators_deviceID))


    def on_error(ws, error):
        print(error)


    def on_close(ws, close_status_code, close_msg):
        print("### websocket closed ###")


    def on_open(ws):
        def run(*args):
            sub_cmd = {
                "tsSubCmds": [
                    {
                        "entityType": "DEVICE",
                        "entityId": sensors_deviceID_str,
                        "scope": "LATEST_TELEMETRY",
                        "cmdId": 1
                    }
                ],
                "historyCmds": [],
                "attrSubCmds": []
            }
            sub_cmd_str = json.dumps(sub_cmd)
            ws.send(sub_cmd_str)
            print("Subscribed to sensors")

        _thread.start_new_thread(run, ())

    try:
        # Auth with credentials
        login_status = rest_client.login(username=username, password=password)
        print("Login done!")

        token = rest_client.configuration.api_key["X-Authorization"]
        if port:
            ws_url = "wss://" + base_url + ":" + port + "/api/ws/plugins/telemetry?token=" + token
        else:
            ws_url = "wss://" + base_url + "/api/ws/plugins/telemetry?token=" + token
        print("Websocket url:", ws_url)
        ws = websocket.WebSocketApp(ws_url, on_open=on_open, on_message=on_message, on_error=on_error, on_close=on_close)

        ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

    except ApiException as e:
        print(e)
