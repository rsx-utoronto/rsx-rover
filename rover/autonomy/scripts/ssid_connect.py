#!/usr/bin/env python3

import subprocess
import argparse
import requests


def check_ssid(ssid: str):
    try:
        ssids = subprocess.check_output(['nmcli', '-t', '-f', 'SSID', 'dev', 'wifi'], universal_newlines=True)
        ssids = ssids.strip().split('\n')
        ssids = {*ssids}

        return True if ssid in ssids else False
    
    except subprocess.CalledProcessError:
        print("Error executing nmcli")
        return False
    
def connect_ssid(ssid: str):
    try:
        subprocess.check_call(['nmcli', 'device', 'wifi', 'connect', ssid])
        print(f"Connected to {ssid}")
    except subprocess.CalledProcessError:
        print(f"Error connecting to {ssid}")


def request_html(host: str, port: str):
    url = f"http://{host}:{port}"
    print(f"Performing get request at {url}")

    response = requests.get(url)
    
    if response.status_code == 200:
        return response.text
    else:
        print(f"Failed to fetch data. Status code: {response.status_code}")
        return None



if __name__ == "__main__":
    # parse args for: SSID, HOST, PORT
    args = argparse.ArgumentParser()
    args.add_argument("--ssid", required=True)
    args.add_argument("--host", required=True)
    args.add_argument("--port", required=True)

    args = args.parse_args()    
    response = request_html(args.host, args.port)
    print(response)
    # if check_ssid(args.ssid): connect_ssid(args.ssid)
