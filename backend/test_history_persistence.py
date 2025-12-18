#!/usr/bin/env python3
"""
Test script to validate chat history persistence across sessions
"""
import asyncio
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from database import SessionLocal, init_db_sync
from services.history_service import HistoryService
from datetime import datetime
import uuid

async def test_history_persistence():
    """
    Test that chat history is properly persisted and retrieved across sessions
    """
    print("Testing chat history persistence across sessions...")

    # Initialize database
    init_db_sync()

    # Create history service instance
    history_service = HistoryService()

    # Create two different sessions
    session1_id = "test_session_1_" + str(uuid.uuid4())
    session2_id = "test_session_2_" + str(uuid.uuid4())

    print(f"Session 1 ID: {session1_id}")
    print(f"Session 2 ID: {session2_id}")

    # Create database session
    db = SessionLocal()

    try:
        # Add some test interactions to session 1
        print("\nAdding interactions to session 1...")
        await history_service.save_interaction(
            db=db,
            user_message="Hello, what is AI?",
            ai_response="AI stands for Artificial Intelligence...",
            mode="global",
            session_id=session1_id
        )

        await history_service.save_interaction(
            db=db,
            user_message="How does machine learning work?",
            ai_response="Machine learning uses algorithms to learn from data...",
            mode="global",
            session_id=session1_id
        )

        print("Added 2 interactions to session 1")

        # Add some test interactions to session 2
        print("\nAdding interactions to session 2...")
        await history_service.save_interaction(
            db=db,
            user_message="What is Python?",
            ai_response="Python is a high-level programming language...",
            mode="selection",
            session_id=session2_id
        )

        print("Added 1 interaction to session 2")

        # Retrieve session 1 history
        print(f"\nRetrieving history for session 1 ({session1_id})...")
        session1_history = await history_service.get_session_history(db, session1_id)
        print(f"Found {len(session1_history)} interactions in session 1")

        for i, interaction in enumerate(session1_history):
            print(f"  Interaction {i+1}:")
            print(f"    User: {interaction.user_message[:50]}...")
            print(f"    AI: {interaction.ai_response[:50]}...")
            print(f"    Mode: {interaction.mode}")
            print(f"    Timestamp: {interaction.timestamp}")

        # Retrieve session 2 history
        print(f"\nRetrieving history for session 2 ({session2_id})...")
        session2_history = await history_service.get_session_history(db, session2_id)
        print(f"Found {len(session2_history)} interactions in session 2")

        for i, interaction in enumerate(session2_history):
            print(f"  Interaction {i+1}:")
            print(f"    User: {interaction.user_message[:50]}...")
            print(f"    AI: {interaction.ai_response[:50]}...")
            print(f"    Mode: {interaction.mode}")
            print(f"    Timestamp: {interaction.timestamp}")

        # Verify that sessions are separate
        print(f"\nVerifying session separation...")
        all_sessions = await history_service.get_all_sessions(db)
        print(f"All session IDs in database: {all_sessions}")

        # Check that we have both sessions
        assert session1_id in all_sessions, f"Session {session1_id} not found in database"
        assert session2_id in all_sessions, f"Session {session2_id} not found in database"
        print("[PASS] Both sessions exist in database")

        # Check that session histories are different
        assert len(session1_history) == 2, f"Expected 2 interactions in session 1, got {len(session1_history)}"
        assert len(session2_history) == 1, f"Expected 1 interaction in session 2, got {len(session2_history)}"
        print("[PASS] Session histories have correct number of interactions")

        # Test session summaries
        print(f"\nTesting session summaries...")
        session1_summary = await history_service.get_session_summary(db, session1_id)
        session2_summary = await history_service.get_session_summary(db, session2_id)

        print(f"Session 1 summary: {session1_summary}")
        print(f"Session 2 summary: {session2_summary}")

        assert session1_summary['message_count'] == 2, f"Expected 2 messages in session 1 summary, got {session1_summary['message_count']}"
        assert session2_summary['message_count'] == 1, f"Expected 1 message in session 2 summary, got {session2_summary['message_count']}"
        print("[PASS] Session summaries are correct")

        print("\n[PASS] All history persistence tests passed!")
        return True

    except Exception as e:
        print(f"\n[FAIL] History persistence test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        db.close()

async def test_multi_session_functionality():
    """
    Test multi-session functionality
    """
    print("\nTesting multi-session functionality...")

    # Initialize database
    init_db_sync()

    # Create history service instance
    history_service = HistoryService()

    # Create multiple sessions
    sessions = []
    for i in range(3):
        session_id = f"multi_test_session_{i+1}_" + str(uuid.uuid4())
        sessions.append(session_id)

    print(f"Created {len(sessions)} test sessions: {sessions}")

    # Create database session
    db = SessionLocal()

    try:
        # Add interactions to each session
        for i, session_id in enumerate(sessions):
            print(f"\nAdding interactions to session {i+1} ({session_id})...")
            for j in range(i+1):  # Different number of messages per session
                await history_service.save_interaction(
                    db=db,
                    user_message=f"Message {j+1} from session {i+1}",
                    ai_response=f"Response {j+1} for session {i+1}",
                    mode="global" if j % 2 == 0 else "selection",
                    session_id=session_id
                )

        # Retrieve all sessions
        print(f"\nRetrieving all sessions...")
        all_sessions = await history_service.get_all_sessions(db)
        print(f"All sessions in database: {all_sessions}")

        # Verify each session has the correct number of messages
        for i, session_id in enumerate(sessions):
            history = await history_service.get_session_history(db, session_id)
            expected_count = i + 1
            actual_count = len(history)
            print(f"Session {session_id}: expected {expected_count}, got {actual_count} messages")
            assert actual_count == expected_count, f"Session {session_id} has {actual_count} messages, expected {expected_count}"

        print("[PASS] Multi-session functionality test passed!")
        return True

    except Exception as e:
        print(f"\n[FAIL] Multi-session functionality test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        db.close()

async def main():
    """
    Main test function
    """
    print("Starting chat history persistence validation tests...\n")

    # Test 1: Basic history persistence
    test1_passed = await test_history_persistence()

    # Test 2: Multi-session functionality
    test2_passed = await test_multi_session_functionality()

    print(f"\nTest Results:")
    print(f"  History Persistence: {'PASS' if test1_passed else 'FAIL'}")
    print(f"  Multi-Session Functionality: {'PASS' if test2_passed else 'FAIL'}")

    if test1_passed and test2_passed:
        print("\n✓ All validation tests passed! Chat history persistence is working correctly.")
        return True
    else:
        print("\n✗ Some validation tests failed.")
        return False

if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)