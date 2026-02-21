"""
Vercel Serverless Function Entry Point
"""
from http.server import BaseHTTPRequestHandler
import json
import os

class handler(BaseHTTPRequestHandler):
    def _set_headers(self, status=200):
        self.send_response(status)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        """Handle GET requests"""
        path = self.path.split('?')[0]
        
        if path == '/' or path == '':
            response = {
                "message": "BiblioChat API is running!",
                "status": "healthy",
                "endpoints": ["/api/health", "/api/chat"]
            }
            self._set_headers(200)
        elif path == '/api/health':
            response = {"status": "healthy", "platform": "local"}
            self._set_headers(200)
        else:
            response = {"error": "Not Found", "path": path}
            self._set_headers(404)
        
        self.wfile.write(json.dumps(response).encode())
        return

    def do_POST(self):
        """Handle POST requests"""
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
                        
                        print("Calling Cohere API...")
                        # Use current supported model (as of 2026)
                        response = co.chat(message=prompt, model="command-a-03-2025")
                        ai_response = response.text
                        print("Cohere response received.")
                    except Exception as e:
                        print(f"Cohere client error: {str(e)}")
                        # If API fails, fallback to demo response
                        ai_response = self.get_demo_response(query)
                else:
                    print("No COHERE_API_KEY found, using demo response.")
                    ai_response = self.get_demo_response(query)
                
                self._set_headers(200)
                response = {"response": ai_response, "sources": []}
                self.wfile.write(json.dumps(response).encode())
                
            except Exception as e:
                print(f"POST error: {str(e)}")
                self._set_headers(500)
                self.wfile.write(json.dumps({"error": str(e)}).encode())
        else:
            self._set_headers(404)
            self.wfile.write(json.dumps({"error": "Not Found"}).encode())
        return

    def do_OPTIONS(self):
        """Handle CORS preflight"""
        self._set_headers(200)
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

if __name__ == "__main__":
    from http.server import HTTPServer
    from dotenv import load_dotenv
    
    # Load environment variables from .env
    load_dotenv()
    
    port = int(os.environ.get("PORT", 8000))
    server = HTTPServer(('0.0.0.0', port), handler)
    print(f"Starting BiblioChat Backend on port {port}...")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping server...")
        server.server_close()
