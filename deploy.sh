#!/bin/bash
# Deployment script for GitHub Pages

# Build the site
cd frontend
npm run build

# Navigate to the build directory
cd build

# Initialize a new git repository
git init

# Add all built files
git add .

# Commit the changes
git commit -m "Deploy to GitHub Pages"

# Set the remote origin to your GitHub repository
# Replace the URL below with your actual GitHub repository URL
git remote add origin https://github.com/your-username/your-repository-name.git

# Push to the gh-pages branch (for project pages) or master branch (for user pages)
git push -f origin main:gh-pages

echo "Deployment to GitHub Pages completed!"