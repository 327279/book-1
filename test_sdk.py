import google.generativeai as genai
import os

key = "AIzaSyAn8KCNhzXsaw8vnLbIeORLR2fOFcXySc4"
genai.configure(api_key=key)

print("Listing models...")
try:
    for m in genai.list_models():
        if 'generateContent' in m.supported_generation_methods:
            print(f"- {m.name}")
except Exception as e:
    print(f"List failed: {e}")

print("\nTesting Generation with gemini-1.5-flash...")
try:
    model = genai.GenerativeModel('gemini-1.5-flash')
    response = model.generate_content("Hi")
    print(f"Response: {response.text}")
except Exception as e:
    print(f"Flash failed: {e}")

print("\nTesting Generation with gemini-pro...")
try:
    model = genai.GenerativeModel('gemini-pro')
    response = model.generate_content("Hi")
    print(f"Pro failed: {response.text}")
except Exception as e:
    print(f"Pro failed: {e}")
