import urllib.request
import json
url = 'https://api.thingspeak.com/channels/330275/feeds.json?api_key=6KPE66DUXE2QJ3I0&results=2'
req = urllib.request.Request(url)

#parsing response
r = urllib.request.urlopen(req).read()
json_data = json.loads(r.decode('utf-8'))
print(json_data['feeds'][-1]['field1'])

