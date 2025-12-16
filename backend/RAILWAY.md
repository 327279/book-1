# Railway Deployment Configuration

This backend can be deployed to Railway.app

## Quick Deploy

1. Install Railway CLI:
```bash
npm install -g @railway/cli
```

2. Login to Railway:
```bash
railway login
```

3. Initialize project:
```bash
railway init
```

4. Add environment variables:
```bash
railway variables set OPENAI_API_KEY=your_key_here
railway variables set DATABASE_URL=your_neon_postgres_url
railway variables set QDRANT_URL=your_qdrant_url
railway variables set QDRANT_API_KEY=your_qdrant_key
```

5. Deploy:
```bash
railway up
```

## Environment Variables Required

- `OPENAI_API_KEY` - OpenAI API key for embeddings and generation
- `DATABASE_URL` - Neon Postgres connection string
- `QDRANT_URL` - Qdrant cloud URL (e.g., https://xxx.qdrant.io)
- `QDRANT_API_KEY` - Qdrant API key
- `MODEL_NAME` - (Optional) Default: gpt-4o-mini
- `EMBEDDING_MODEL_NAME` - (Optional) Default: text-embedding-3-small

## Database Setup

The application will automatically create tables on first run.

## Post-Deployment

1. Run document ingestion:
```bash
railway run python ingest_documents.py
```

2. Test the API:
```bash
curl https://your-app.railway.app/docs
```

## Start Command

Railway will automatically detect the start command from `Procfile`:
```
uvicorn src.api.main:app --host 0.0.0.0 --port $PORT
```
