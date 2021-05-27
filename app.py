#!/usr/bin/python3
import sys
import time
import logging
import random
from opcua import Client,ua

sys.path.append('/usr/local/lib/python3.7/dist-packages/opcua')



if __name__ == "__main__":
    logging.basicConfig(level=logging.WARN)
    logger = logging.getLogger("KeepAlive")

    #client = Client("opc.tcp://192.168.198.94:4840")
    client = Client("opc.tcp://149.220.77.141:4840")
    client.connect()
    client.socket()
    client.load_type_definitions()
    root = client.get_root_node()
    while True:
        temp =     client.get_node("ns=1;s=temperature_bme")
        humidity = client.get_node("ns=1;s=humidity_bme")
        pressure = client.get_node("ns=1;s=pressure_bme")
        print(temp)
        print(temp.get_value())
        print(humidity)
        print(humidity.get_value())
        print(pressure)
        print(pressure.get_value())
        time.sleep(1)
    client.disconnect()


