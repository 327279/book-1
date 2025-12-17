# Vercel entrypoint - exports the FastAPI app
import sys
from pathlib import Path

# Add the backend directory to Python path for proper imports
backend_dir = Path(__file__).parent.parent
sys.path.insert(0, str(backend_dir))

# Now import the app
from src.api.main import app

# Vercel looks for 'app' in this file
__all__ = ["app"]
