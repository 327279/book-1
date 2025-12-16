from fastapi.testclient import TestClient
from src.api.main import app

client = TestClient(app)

def test_health_check():
    response = client.get("/api/v1/queries/health")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy", "message": "RAG API is running"}

def test_create_query_missing_text():
    response = client.post("/api/v1/queries/", params={"query": ""})
    assert response.status_code == 400
