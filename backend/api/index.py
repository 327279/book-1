"""
Vercel Serverless Function Entry Point
"""
from http.server import BaseHTTPRequestHandler
import json
import os

class handler(BaseHTTPRequestHandler):
    def do_GET(self):
        """Handle GET requests"""
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        
        path = self.path.split('?')[0]  # Remove query params
        
        if path == '/' or path == '':
            response = {
                "message": "BiblioChat API is running on Vercel!",
                "status": "healthy",
                "endpoints": ["/api/health", "/api/chat"]
            }
        elif path == '/api/health':
            response = {"status": "healthy", "platform": "vercel"}
        elif path == '/favicon.ico':
            # Return empty response for favicon
            response = {}
        else:
            response = {"error": "Not Found", "path": path}
        
        self.wfile.write(json.dumps(response).encode())
        return

    def do_POST(self):
        """Handle POST requests"""
        self.send_header('Access-Control-Allow-Origin', '*')
        
        if self.path == '/api/chat':
            try:
                content_length = int(self.headers.get('Content-Length', 0))
                post_data = self.rfile.read(content_length)
                request_body = json.loads(post_data.decode())
                
                query = request_body.get("query", "")
                selection_context = request_body.get("selection_context")
                
                # Try to use Cohere
                cohere_api_key = os.environ.get("COHERE_API_KEY")
                
                if cohere_api_key:
                    try:
                        import cohere
                        co = cohere.Client(api_key=cohere_api_key)
                        
                        if selection_context:
                            prompt = f"You are a helpful robotics assistant. Analyze this text: {selection_context}. Question: {query}"
                        else:
                            prompt = f"You are a helpful robotics assistant for a Physical AI & Humanoid Robotics textbook. Question: {query}"
                        
                        response = co.chat(message=prompt, model="command-r")
                        ai_response = response.text
                    except Exception as e:
                        ai_response = f"Cohere error: {str(e)}"
                else:
                    # Demo response if no API key
                    ai_response = self.get_demo_response(query)
                
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                
                response = {"response": ai_response, "sources": []}
                self.wfile.write(json.dumps(response).encode())
                
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode())
        else:
            self.send_response(404)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"error": "Not Found"}).encode())
        return

    def do_OPTIONS(self):
        """Handle CORS preflight"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
        return
    
    def get_demo_response(self, query):
        """Demo responses when API key not configured"""
        q = query.lower()
        if 'ros' in q:
            return "ROS 2 is the robotics middleware framework providing communication infrastructure for robot applications."
        elif 'simulation' in q or 'gazebo' in q:
            return "Simulation allows testing robots virtually. Gazebo is a physics-based simulator for robotics."
        elif 'isaac' in q or 'nvidia' in q:
            return "NVIDIA Isaac is a platform for accelerated robotics development with photorealistic simulation."
        elif 'vla' in q:
            return "VLA (Vision-Language-Action) models bridge natural language with robotic actions."
        else:
            return "I'm the Physical AI & Robotics Assistant! Ask about ROS 2, simulation, NVIDIA Isaac, or VLA."
