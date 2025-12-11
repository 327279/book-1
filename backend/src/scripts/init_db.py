#!/usr/bin/env python3
"""
Script to initialize the database tables
"""

import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ..config.database import engine, Base
from ..models.document import Document
from ..models.embedding import Embedding
from ..models.query import Query
from ..models.query_result import QueryResult
from ..models.user_session import UserSession
from ..models.code_example import CodeExample


def init_db():
    """
    Create all database tables
    """
    print("Creating database tables...")

    # Create all tables defined in the models using the Base
    Base.metadata.create_all(bind=engine)

    print("Database tables created successfully!")


if __name__ == "__main__":
    init_db()