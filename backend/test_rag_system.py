"""
Test script for verifying RAG system functionality and accuracy
Tests topic detection and query responses for all chapter topics
"""

import requests
import json
from typing import List, Dict

# Test queries for each topic
TEST_QUERIES = {
    "ros2": [
        "What is a ROS 2 node?",
        "How do I create a publisher in rclpy?",
        "Explain ROS 2 topics and services",
        "What is URDF used for in robotics?"
    ],
    "simulation": [
        "How does Gazebo simulate physics?",
        "What sensors can be simulated in Gazebo?",
        "Explain Unity for robot simulation",
        "How to simulate LiDAR sensors?"
    ],
    "isaac": [
        "What is NVIDIA Isaac Sim?",
        "How does Nav2 work for path planning?",
        "Explain Isaac ROS for VSLAM",
        "What are the benefits of Isaac tools?"
    ],
    "vla": [
        "What is a VLA system?",
        "How does Whisper integrate with robotics?",
        "Explain voice-to-action translation",
        "How do cognitive planning systems work?"
    ],
    "capstone": [
        "What is the autonomous humanoid capstone project?",
        "How to integrate voice commands with manipulation?",
        "Explain the capstone object identification system",
        "What components are in the capstone project?"
    ],
    "conversational": [
        "What is conversational robotics?",
        "How do dialogue systems work?",
        "Explain human-robot interaction",
        "What are conversational AI techniques?"
    ],
    "humanoid": [
        "What are humanoid robots?",
        "How do bipedal robots maintain balance?",
        "Explain humanoid locomotion",
        "What sensors do humanoid robots need?"
    ]
}

def test_rag_query(base_url: str, query: str, topic: str) -> Dict:
    """
    Test a single RAG query
    """
    endpoint = f"{base_url}/api/v1/queries/"
    payload = {
        "query_text": query,
        "selected_text": None,
        "session_id": "test-session"
    }
    
    try:
        response = requests.post(endpoint, json=payload)
        if response.status_code == 200:
            result = response.json()
            return {
                "query": query,
                "topic": topic,
                "success": True,
                "response": result.get("response_text", ""),
                "confidence": result.get("confidence_score", 0),
                "sources": result.get("source_documents_json", "")
            }
        else:
            return {
                "query": query,
                "topic": topic,
                "success": False,
                "error": f"Status code: {response.status_code}"
            }
    except Exception as e:
        return {
            "query": query,
            "topic": topic,
            "success": False,
            "error": str(e)
        }

def run_rag_tests(base_url: str = "http://localhost:8000") -> Dict:
    """
    Run all RAG system tests
    """
    results = {
        "total_queries": 0,
        "successful_queries": 0,
        "failed_queries": 0,
        "average_confidence": 0.0,
        "topic_results": {}
    }
    
    total_confidence = 0
    
    for topic, queries in TEST_QUERIES.items():
        topic_results = []
        for query in queries:
            result = test_rag_query(base_url, query, topic)
            topic_results.append(result)
            results["total_queries"] += 1
            
            if result["success"]:
                results["successful_queries"] += 1
                total_confidence += result.get("confidence", 0)
            else:
                results["failed_queries"] += 1
        
        results["topic_results"][topic] = topic_results
    
    if results["successful_queries"] > 0:
        results["average_confidence"] = total_confidence / results["successful_queries"]
    
    results["accuracy_percentage"] = (results["successful_queries"] / results["total_queries"] * 100) if results["total_queries"] > 0 else 0
    
    return results

def print_test_report(results: Dict):
    """
    Print a formatted test report
    """
    print("=" * 80)
    print("RAG SYSTEM TEST REPORT")
    print("=" * 80)
    print(f"\nTotal Queries Tested: {results['total_queries']}")
    print(f"Successful Queries: {results['successful_queries']}")
    print(f"Failed Queries: {results['failed_queries']}")
    print(f"Success Rate: {results['accuracy_percentage']:.2f}%")
    print(f"Average Confidence Score: {results['average_confidence']:.2f}")
    
    print("\n" + "=" * 80)
    print("TOPIC-SPECIFIC RESULTS")
    print("=" * 80)
    
    for topic, topic_queries in results["topic_results"].items():
        print(f"\n{topic.upper()}:")
        successful = sum(1 for q in topic_queries if q["success"])
        total = len(topic_queries)
        print(f"  Success Rate: {successful}/{total} ({successful/total*100:.1f}%)")
        
        for query_result in topic_queries:
            status = "✓" if query_result["success"] else "✗"
            print(f"  {status} {query_result['query'][:60]}...")
            if not query_result["success"]:
                print(f"     Error: {query_result.get('error', 'Unknown error')}")
    
    print("\n" + "=" * 80)
    if results['accuracy_percentage'] >= 90:
        print("✓ PASSED: RAG system meets 90% accuracy requirement")
    else:
        print(f"✗ FAILED: RAG system accuracy ({results['accuracy_percentage']:.2f}%) below 90% requirement")
    print("=" * 80)

if __name__ == "__main__":
    import sys
    
    base_url = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:8000"
    
    print(f"Testing RAG system at: {base_url}")
    print("Running tests...\n")
    
    results = run_rag_tests(base_url)
    print_test_report(results)
    
    # Save results to JSON file
    with open("rag_test_results.json", "w") as f:
        json.dump(results, f, indent=2)
    
    print(f"\nDetailed results saved to: rag_test_results.json")
    
    # Exit with appropriate code
    sys.exit(0 if results['accuracy_percentage'] >= 90 else 1)
