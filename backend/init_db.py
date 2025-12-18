#!/usr/bin/env python3
"""
Database initialization script for BiblioChat backend
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from database import init_db
from vector_setup import VectorSetup

def main():
    """
    Initialize all database components:
    1. Create SQL tables
    2. Set up Qdrant collection
    """
    print("Initializing database components...")

    # Initialize SQL database
    print("Creating SQL tables...")
    init_db()
    print("✓ SQL tables created successfully")

    # Initialize vector database
    print("Setting up Qdrant collection...")
    vector_setup = VectorSetup()
    vector_setup.setup_collection()
    print("✓ Qdrant collection created successfully")

    print("\n✓ All database components initialized successfully!")

if __name__ == "__main__":
    main()