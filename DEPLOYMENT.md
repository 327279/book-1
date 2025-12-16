# Production Deployment Guide

## Overview

This guide covers deploying the Physical AI & Humanoid Robotics textbook to production.

**Frontend**: Vercel (Already configured)  
**Backend**: Railway, Heroku, or Vercel

---

## Prerequisites

- [ ] OpenAI API key
- [ ] Neon Postgres database
- [ ] Qdrant Cloud account
- [ ] Git repository
- [ ] Domain name (optional)

---

## Step 1: Setup External Services

### 1.1 Neon Postgres

1. Go to https://neon.tech
2. Create a new project
3. Copy the connection string: `postgresql://user:pass@host/database`

### 1.2 Qdrant Cloud

1. Go to https://cloud.qdrant.io
2. Create a free cluster
3. Copy the URL and API key

### 1.3 OpenAI

1. Go to https://platform.openai.com
2. Create an API key
3. Add credits to your account

---

## Step 2: Deploy Backend

### Option A: Railway (Recommended)

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login
railway login

# Navigate to backend
cd backend

# Initialize project
railway init

# Set environment variables
railway variables set OPENAI_API_KEY=sk-...
railway variables set DATABASE_URL=postgresql://...
railway variables set QDRANT_URL=https://...
railway variables set QDRANT_API_KEY=...

# Deploy
railway up

# Run migrations and ingestion
railway run python ingest_documents.py
```

Your backend will be at: `https://your-project.railway.app`

### Option B: Heroku

```bash
# Login to Heroku
heroku login

# Create app
cd backend
heroku create your-app-name

# Set environment variables
heroku config:set OPENAI_API_KEY=sk-...
heroku config:set DATABASE_URL=postgresql://...
heroku config:set QDRANT_URL=https://...
heroku config:set QDRANT_API_KEY=...

# Deploy
git push heroku main

# Run ingestion
heroku run python ingest_documents.py
```

### Option C: Vercel (Serverless)

```bash
cd backend

# Install Vercel CLI
npm install -g vercel

# Deploy
vercel

# Set environment variables in dashboard
# Add: OPENAI_API_KEY, DATABASE_URL, QDRANT_URL, QDRANT_API_KEY
```

---

## Step 3: Update Frontend Configuration

### 3.1 Update API Endpoint

Edit `frontend/src/components/ChatBot/ChatBot.jsx`:

```javascript
const BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'https://your-backend.railway.app';
```

### 3.2 Set Environment Variable

Create `frontend/.env.production`:

```bash
REACT_APP_BACKEND_URL=https://your-backend.railway.app
```

### 3.3 Deploy Frontend to Vercel

Frontend is already configured for Vercel. Just push to main:

```bash
git add .
git commit -m "Update backend URL"
git push origin main
```

Vercel will automatically deploy.

---

## Step 4: Configure CORS

Update `backend/src/api/main.py` with your frontend URL:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://book-1-eight.vercel.app",
        "http://localhost:3000"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

Redeploy backend after this change.

---

## Step 5: Ingest Documents

Run the ingestion script to populate the RAG system:

```bash
# Railway
railway run python ingest_documents.py

# Heroku
heroku run python ingest_documents.py

# Vercel (run locally with production DB)
DATABASE_URL=your_prod_db python ingest_documents.py
```

Expected output:
```
Successfully ingested: 18
Failed: 0
```

---

## Step 6: Verification

### 6.1 Test Backend API

```bash
curl https://your-backend.railway.app/docs
```

Should show FastAPI documentation.

### 6.2 Test RAG Query

```bash
curl -X POST https://your-backend.railway.app/api/v1/queries/ \
  -H "Content-Type: application/json" \
  -d '{"query_text": "What is ROS 2?", "session_id": "test"}'
```

Should return a response with RAG answer.

### 6.3 Test Frontend

1. Visit https://book-1-eight.vercel.app
2. Click on any chapter
3. Open the chatbot
4. Ask: "What is this chapter about?"
5. Verify you get a response

---

## Step 7: Monitoring

### Backend Logs

```bash
# Railway
railway logs

# Heroku
heroku logs --tail
```

### Frontend Analytics

- Vercel Dashboard: https://vercel.com/dashboard
- View deployment logs and analytics

---

## Environment Variables Summary

### Backend
```
OPENAI_API_KEY=sk-...
DATABASE_URL=postgresql://...
QDRANT_URL=https://...
QDRANT_API_KEY=...
MODEL_NAME=gpt-4o-mini
EMBEDDING_MODEL_NAME=text-embedding-3-small
```

### Frontend
```
REACT_APP_BACKEND_URL=https://your-backend.railway.app
```

---

## Troubleshooting

### Backend won't start
- Check environment variables are set
- Verify DATABASE_URL is correct
- Check Railway/Heroku logs

### CORS errors
- Verify frontend URL in CORS configuration
- Redeploy backend after updating CORS

### RAG returns empty responses
- Check if documents are ingested
- Verify OPENAI_API_KEY has credits
- Check Qdrant connection

### Database connection errors
- Verify Neon Postgres is running
- Check DATABASE_URL format
- Ensure IP whitelist includes Railway/Heroku IPs

---

## Cost Estimation

**Neon Postgres**: Free tier (512MB storage)  
**Qdrant Cloud**: Free tier (1GB storage)  
**OpenAI API**: ~$0.10 per 1000 queries  
**Railway**: ~$5/month (with free tier initially)  
**Vercel**: Free for frontend

**Total**: ~$5-10/month for moderate usage

---

## Next Steps

- [ ] Set up custom domain
- [ ] Configure SSL certificates
- [ ] Set up monitoring and alerts
- [ ] Configure backup strategy
- [ ] Set up CI/CD pipeline
- [ ] Add rate limiting
- [ ] Implement caching

---

## Support

For issues, check:
- Railway: https://railway.app/help
- Vercel: https://vercel.com/docs
- Heroku: https://devcenter.heroku.com
