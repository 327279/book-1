# Vercel Deployment Configuration

This backend can be deployed to Vercel.

## Quick Deploy

### Option 1: Deploy via Vercel CLI

1. Install Vercel CLI:
```bash
npm install -g vercel
```

2. Login to Vercel:
```bash
vercel login
```

3. Deploy from the backend directory:
```bash
cd backend
vercel
```

4. Add environment variables in Vercel Dashboard:
   - Go to your project settings → Environment Variables
   - Add the following variables:
     - `OPENAI_API_KEY`
     - `DATABASE_URL`
     - `QDRANT_URL`
     - `QDRANT_API_KEY`

5. Redeploy after adding environment variables:
```bash
vercel --prod
```

### Option 2: Deploy via GitHub

1. Push your code to GitHub
2. Go to [vercel.com](https://vercel.com)
3. Click "Import Project"
4. Select your repository
5. Set the Root Directory to `backend`
6. Add environment variables in the deployment settings
7. Deploy

## Environment Variables Required

| Variable | Description |
|----------|-------------|
| `OPENAI_API_KEY` | OpenAI API key for embeddings and generation |
| `DATABASE_URL` | Neon Postgres connection string |
| `QDRANT_URL` | Qdrant cloud URL (e.g., https://xxx.qdrant.io) |
| `QDRANT_API_KEY` | Qdrant API key |
| `MODEL_NAME` | (Optional) Default: gpt-4o-mini |
| `EMBEDDING_MODEL_NAME` | (Optional) Default: text-embedding-3-small |

## Important Notes

### Serverless Limitations
- Vercel runs Python as serverless functions
- Each request has a maximum execution time (10s on Hobby, 60s on Pro)
- Cold starts may occur after inactivity

### Database Connections
- Use connection pooling for Postgres (Neon provides this)
- Qdrant Cloud handles connections automatically

### Post-Deployment

1. Test the API:
```bash
curl https://your-app.vercel.app/
curl https://your-app.vercel.app/health
curl https://your-app.vercel.app/docs
```

2. Run document ingestion locally (pointing to production services):
```bash
# Set production environment variables locally
export DATABASE_URL="your_production_db_url"
export QDRANT_URL="your_production_qdrant_url"
export QDRANT_API_KEY="your_production_qdrant_key"
export OPENAI_API_KEY="your_openai_key"

python ingest_documents.py
```

## Troubleshooting

### Build Failures
- Check that all dependencies in `requirements.txt` are compatible with Python 3.11
- Ensure `runtime.txt` specifies `python-3.11.0`

### 500 Errors
- Check Vercel Function logs in the dashboard
- Verify all environment variables are set correctly

### CORS Issues
- The API allows all origins by default
- For production, update `allow_origins` in `main.py` to your specific frontend URL
