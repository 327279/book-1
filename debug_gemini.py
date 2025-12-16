import urllib.request
import json
import sys

print("Starting debug script...")
try:
    url = 'https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key=AIzaSyAn8KCNhzXsaw8vnLbIeORLR2fOFcXySc4'
    data = json.dumps({'contents': [{'parts': [{'text': 'Hello'}]}]}).encode('utf-8')
    headers = {'Content-Type': 'application/json'}
    
    print(f"Requesting URL: {url}")
    req = urllib.request.Request(url, data=data, headers=headers)
    
    print("Sending request (timeout=10s)...")
    with urllib.request.urlopen(req, timeout=10) as response:
        print("Response received!")
        print(response.read().decode('utf-8'))
        
except Exception as e:
    print(f"ERROR: {e}")
print("Script finished.")
