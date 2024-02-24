import os
import requests


ssid = ""
password = ""

# CONNECTING TO THE SUIT WIFI ===========================================================================================
wifi_scanner = os.system(f'nmcli dev wifi connect {ssid} password {password}')
while wifi_scanner == 2560:
    wifi_scanner = os.system(f'nmcli dev wifi connect {ssid} password {password}')

print("Connected")

# WEB SCRAPING TO GET THE LOCATION ============================================================================
url = 'http://127.0.0.1:5500/catkin_ros_1/src/wifi_connection/scripts/testing_page.html'
html_page = requests.get(url).text
print(html_page)
data = html_page.split("<br>")
for line in data:
    if 'Location' in line:
        location = line
location = data[3].split(" ")
first, second = float(location[1].replace('N,', '')), float(location[2].replace('W\r\n', ''))
print((first, second))