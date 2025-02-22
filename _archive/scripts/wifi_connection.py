import os
import requests


ssid = "Isaac_Asimov"

# CONNECTING TO THE SUIT WIFI ===========================================================================================
wifi_scanner = os.system(f'nmcli dev wifi connect {ssid}')
while wifi_scanner == 2560:
    wifi_scanner = os.system(f'nmcli dev wifi connect {ssid}')

print("Connected")

# WEB SCRAPING TO GET THE LOCATION ============================================================================
url = 'http://10.10.11.1:80'
html_page = requests.get(url).text
print(html_page)
data = html_page.split("<br>")
for line in data:
    if 'Location' in line:
        location = line
location = data[3].split(" ")
first, second = float(location[1].replace('N,', '')), float(location[2].replace('W\r\n', ''))
print((first, second))
