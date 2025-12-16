#!/bin/bash
# Local Verification Script
# Tests that everything works before deployment

echo "========================================="
echo "Physical AI Textbook - Local Verification"
echo "========================================="
echo ""

# Check if we're in the correct directory
if [ ! -d "frontend" ] || [ ! -d "backend" ]; then
    echo "❌ Error: Must run from project root directory"
    exit 1
fi

echo "✓ Project structure verified"
echo ""

# Test 1: Frontend Build
echo "Test 1: Frontend Build"
echo "----------------------"
cd frontend
if npm run build > /dev/null 2>&1; then
    echo "✓ Frontend builds successfully"
else
    echo "❌ Frontend build failed"
    exit 1
fi
cd ..
echo ""

# Test 2: Backend Dependencies
echo "Test 2: Backend Dependencies"
echo "----------------------------"
cd backend
if [ -d "venv" ]; then
    source venv/bin/activate 2>/dev/null || . venv/Scripts/activate 2>/dev/null
    if pip freeze | grep -q "fastapi"; then
        echo "✓ Backend dependencies installed"
    else
        echo "⚠️  Warning: FastAPI not found in venv, installing..."
        pip install -r requirements.txt > /dev/null 2>&1
    fi
else
    echo "⚠️  Warning: Virtual environment not found"
    echo "   Run: python -m venv venv && pip install -r requirements.txt"
fi
cd ..
echo ""

# Test 3: Check for required files
echo "Test 3: Required Files"
echo "----------------------"
required_files=(
    "backend/requirements.txt"
    "backend/Procfile"
    "backend/ingest_documents.py"
    "backend/test_rag_system.py"
    "frontend/package.json"
    "frontend/docusaurus.config.js"
    "README.md"
    "DEPLOYMENT.md"
)

all_files_exist=true
for file in "${required_files[@]}"; do
    if [ -f "$file" ]; then
        echo "✓ $file"
    else
        echo "❌ Missing: $file"
        all_files_exist=false
    fi
done
echo ""

# Test 4: Check chapters exist
echo "Test 4: Chapter Files"
echo "---------------------"
chapter_count=$(ls -1 frontend/docs/chapter-*.md 2>/dev/null | wc -l)
if [ "$chapter_count" -ge 18 ]; then
    echo "✓ Found $chapter_count chapter files"
else
    echo "⚠️  Warning: Only found $chapter_count chapter files (expected 18)"
fi
echo ""

# Test 5: Check example directories
echo "Test 5: Code Examples"
echo "--------------------"
example_dirs=("ros2" "simulation" "isaac" "vla" "capstone")
for dir in "${example_dirs[@]}"; do
    if [ -d "frontend/docs/examples/$dir" ]; then
        echo "✓ examples/$dir"
    else
        echo "❌ Missing: examples/$dir"
    fi
done
echo ""

# Summary
echo "========================================="
echo "VERIFICATION SUMMARY"
echo "========================================="
if [ "$all_files_exist" = true ] && [ "$chapter_count" -ge 18 ]; then
    echo "✓ All checks passed!"
    echo ""
    echo "Next steps:"
    echo "1. Test backend locally:"
    echo "   cd backend && uvicorn src.api.main:app --reload"
    echo ""
    echo "2. Test frontend locally:"
    echo "   cd frontend && npm run start"
    echo ""
    echo "3. Deploy to production (see DEPLOYMENT.md)"
    exit 0
else
    echo "⚠️  Some checks failed. Please review above."
    exit 1
fi
