# Quick Railway Deployment Guide

Since Railway CLI init had issues, here's the manual deployment approach:

## Option 1: Railway Web Dashboard (Recommended)

1. **Go to Railway Dashboard**
   - Visit: https://railway.app/dashboard
   - Click "New Project"

2. **Deploy from GitHub**
   - Click "Deploy from GitHub repo"
   - Select your `book-1` repository
   - Railway will auto-detect the Python app

3. **Configure Root Directory**
   - In project settings, set root directory to: `backend`
   - Railway should detect the `Procfile`

4. **Add Environment Variables**
   Copy these from your `.env` file:
   - `OPENAL_API_KEY` = your_openai_key
   - `DATABASE_URL` = your_neon_postgres_url
   - `QDRANT_URL` = your_qdrant_url
   - `QDRANT_API_KEY` = your_qdrant_key
   - `MODEL_NAME` = gpt-4o-mini
   - `EMBEDDING_MODEL_NAME` = text-embedding-3-small

5. **Deploy**
   - Click "Deploy"
   - Railway will build and deploy automatically
   - You'll get a URL like: `https://your-app.railway.app`

6. **Run Document Ingestion**
   After deployment, in Railway dashboard:
   - Go to your service
   - Click "···" menu → "Run a Command"
   - Run: `python ingest_documents.py`

## Option 2: Vercel (Serverless - Fastest)

Since you have the backend folder ready:

```bash
cd backend
npm install -g vercel
vercel
```

Follow prompts and add environment variables in Vercel dashboard.

## Option 3: Use the Prepared Commands

I've prepared everything. Just need to:
1. Push to GitHub (if not already)
2. Connect Railway to your GitHub repo
3. Add environment variables
4. Deploy!

The backend is 100% ready for deployment! 🚀
