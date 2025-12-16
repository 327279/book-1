import requests
import json
import sys

models_to_test = [
    "gemini-1.5-flash",
    "gemini-1.5-flash-001",
    "gemini-1.5-pro",
    "gemini-pro",
    "gemini-1.0-pro"
]

versions = ["v1beta", "v1"]

print("Starting multi-model debug with REQUESTS...")
headers = {'Content-Type': 'application/json'}
data = {'contents': [{'parts': [{'text': 'Hello'}]}]}

success = False

for version in versions:
    for model in models_to_test:
        url = f"https://generativelanguage.googleapis.com/{version}/models/{model}:generateContent?key=AIzaSyAn8KCNhzXsaw8vnLbIeORLR2fOFcXySc4"
        print(f"Testing {version} / {model} ...")
        try:
            response = requests.post(url, json=data, headers=headers, timeout=5)
            if response.status_code == 200:
                print(f"SUCCESS! {model} ({version}) works!")
                print("Response snippet:", response.text[:100])
                success = True
                break
            else:
                print(f"Failed: {response.status_code} {response.text[:50]}")
        except Exception as e:
            print(f"Error: {e}")
    if success: break

if not success:
    print("ALL FAILED.")
