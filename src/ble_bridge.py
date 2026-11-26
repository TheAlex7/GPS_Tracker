"""
BLE bridge: GPS tracker (ESP32, "IoT-T09-GPS") <-> Node-RED on this computer.

Setup (once):   pip install bleak
Run:            python ble_bridge.py

- Connects to the tracker over Bluetooth and reconnects by itself if the link drops.
- Sends JSON lines to Node-RED over UDP 127.0.0.1:5005 (same format as the tracker's
  "json on" serial output): {"type":"loc",...}, {"type":"alert",...}, {"type":"log",...},
  plus {"type":"bridge",...} for connection changes and sent commands.
- Listens for commands from Node-RED on UDP 127.0.0.1:5006 (plain text):
      fence 33.6405,-117.8443,50   fence PHONE,50   fence OFF
      find 1   find 0
      phone 33.6405,-117.8443
      led 1    led 0
"""

import asyncio
import json
import socket

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "IoT-T09-GPS"
NODE_RED_ADDRESS = ("127.0.0.1", 5005)
COMMAND_ADDRESS = ("127.0.0.1", 5006)
FENCE_REFRESH_SECONDS = 10

def uuid(n):
    return f"83f1000{n}-8ca4-4c09-bb11-7357ac492601"

LAT_UUID, LON_UUID, STATUS_UUID = uuid(1), uuid(2), uuid(3)
FENCE_CFG_UUID, FENCE_STATE_UUID, ALERT_UUID = uuid(4), uuid(5), uuid(6)
PHONE_POS_UUID, FIND_UUID, LOG_UUID = uuid(7), uuid(8), uuid(9)
LED_UUID = "499306f5-0f14-4d5d-9d31-bdd52c597914"

COMMAND_TARGETS = {"fence": FENCE_CFG_UUID, "find": FIND_UUID, "phone": PHONE_POS_UUID, "led": LED_UUID}

ALERT_EVENTS = [
    ("ALERT: OUTSIDE FENCE", "exit"),
    ("BACK INSIDE", "enter"),
    ("ALERT: PHONE OUT OF RANGE", "phone_lost"),
    ("PHONE BACK IN RANGE", "phone_back"),
    ("ALERT: GPS OFFLINE", "gps_offline"),
    ("GPS BACK ONLINE", "gps_online"),
]

udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state = {"lat": None, "lon": None, "status": None, "sats": None,
         "fence": None, "fence_cfg": "NOT SET", "find": False}
seen_log_lines = []


def send(message):
    udp.sendto((json.dumps(message) + "\n").encode("utf-8"), NODE_RED_ADDRESS)


def text(data):
    return bytes(data).decode("utf-8", "replace").strip()


def number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def fence_from_config(config):
    """'lat,lon,r' -> custom fence; 'PHONE,r' -> phone mode (centre unknown here); else off."""
    parts = config.split(",")
    if len(parts) == 3:
        return "custom", number(parts[0]), number(parts[1]), number(parts[2])
    if len(parts) == 2 and parts[0] == "PHONE":
        return "phone", None, None, number(parts[1])
    return "off", None, None, None


def location_message():
    mode, fence_lat, fence_lon, fence_r = fence_from_config(state["fence_cfg"])
    if fence_lat is None:  # no centre to draw (off, or phone mode)
        fence_r = None
    return {"type": "loc", "lat": state["lat"], "lon": state["lon"], "sats": state["sats"],
            "status": state["status"], "fence": state["fence"], "fence_mode": mode,
            "fence_lat": fence_lat, "fence_lon": fence_lon, "fence_r": fence_r,
            "find": state["find"], "ble": True, "t": None}


# ---- Notification handlers ----

def on_lat(_, data):
    state["lat"] = number(text(data))  # "--" before the first fix -> None


def on_lon(_, data):
    state["lon"] = number(text(data))


def on_status(_, data):
    # "FIX SATS:7" -> status "FIX", sats 7
    status, _, sats = text(data).partition(" SATS:")
    state["status"] = status
    state["sats"] = int(sats) if sats.isdigit() else None


def on_fence_state(_, data):
    # The tracker sends this last in its once-a-second batch, so publish the location now.
    state["fence"] = text(data)
    send(location_message())


def on_alert(_, data):
    message = text(data)
    event = next((code for prefix, code in ALERT_EVENTS if message.startswith(prefix)), "other")
    send({"type": "alert", "event": event, "msg": message,
          "lat": state["lat"], "lon": state["lon"], "t": None})


def on_find(_, data):
    state["find"] = text(data).startswith("ON")


def publish_new_log_lines(value):
    global seen_log_lines
    lines = [line for line in value.splitlines() if line.strip()]
    for line in lines:
        if line not in seen_log_lines:
            send({"type": "log", "msg": line})
    seen_log_lines = lines


# ---- Main loop ----

async def read_fence_config(client):
    state["fence_cfg"] = text(await client.read_gatt_char(FENCE_CFG_UUID))


async def handle_command(client, command):
    name, _, argument = command.partition(" ")
    target = COMMAND_TARGETS.get(name.lower())
    if target is None or not argument:
        print("Unknown command:", command)
        send({"type": "bridge", "error": "unknown command", "command": command})
        return
    await client.write_gatt_char(target, argument.encode("utf-8"), response=True)
    if target == FENCE_CFG_UUID:
        await read_fence_config(client)
    print("Sent:", command)
    send({"type": "bridge", "sent": command})


async def main():
    loop = asyncio.get_running_loop()
    commands = asyncio.Queue()

    class CommandProtocol(asyncio.DatagramProtocol):
        def datagram_received(self, data, addr):
            commands.put_nowait(data.decode("utf-8", "replace").strip())

    await loop.create_datagram_endpoint(CommandProtocol, local_addr=COMMAND_ADDRESS)
    print(f"Forwarding to Node-RED on UDP {NODE_RED_ADDRESS[1]}, commands on UDP {COMMAND_ADDRESS[1]}")

    while True:
        print("Scanning for", DEVICE_NAME, "...")
        device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10)
        if device is None:
            continue
        disconnected = asyncio.Event()
        try:
            async with BleakClient(device, disconnected_callback=lambda _: disconnected.set()) as client:
                print("Connected to", device.address)
                send({"type": "bridge", "connected": True})
                while not commands.empty():  # drop commands sent while disconnected
                    commands.get_nowait()

                async def read_log():
                    try:
                        publish_new_log_lines(text(await client.read_gatt_char(LOG_UUID)))
                    except Exception as error:
                        print("Log read failed:", error)

                def on_log(_, data):
                    # A notification may be cut short at the BLE packet size, so read the full value.
                    asyncio.ensure_future(read_log())

                await read_fence_config(client)
                await read_log()
                for char_uuid, handler in [(LAT_UUID, on_lat), (LON_UUID, on_lon), (STATUS_UUID, on_status),
                                           (FENCE_STATE_UUID, on_fence_state), (ALERT_UUID, on_alert),
                                           (FIND_UUID, on_find), (LOG_UUID, on_log)]:
                    await client.start_notify(char_uuid, handler)

                while not disconnected.is_set():
                    try:
                        command = await asyncio.wait_for(commands.get(), timeout=FENCE_REFRESH_SECONDS)
                    except asyncio.TimeoutError:
                        await read_fence_config(client)
                        continue
                    await handle_command(client, command)
        except Exception as error:
            print("BLE error:", error)
        print("Disconnected; retrying in 3 s")
        send({"type": "bridge", "connected": False})
        await asyncio.sleep(3)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
