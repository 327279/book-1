/**
 * API Configuration
 * Uses environment variable for backend URL, falls back to localhost for development
 */

// Backend API base URL - Set VITE_API_URL in Vercel environment variables
export const API_BASE_URL = typeof window !== 'undefined'
    ? (window.__ENV__?.API_URL || process.env.VITE_API_URL || 'http://localhost:8000')
    : 'http://localhost:8000';

// Check if we're in production (Vercel deployment)
export const IS_PRODUCTION = typeof window !== 'undefined' &&
    window.location.hostname !== 'localhost';

// Demo mode - set to false when backend is deployed
// Set VITE_DEMO_MODE=false in Vercel to enable real API calls
export const DEMO_MODE = process.env.VITE_DEMO_MODE === 'true' ||
    (IS_PRODUCTION && !process.env.VITE_API_URL);

// API endpoints
export const API_ENDPOINTS = {
    queries: `${API_BASE_URL}/api/v1/queries/`,
    agentSkill: `${API_BASE_URL}/api/v1/agent/skill`,
    userProfile: `${API_BASE_URL}/api/v1/users/profile`,
    auth: `${API_BASE_URL}/api/auth`,
};

/**
 * Safe fetch wrapper that handles backend unavailability gracefully
 */
export async function safeFetch(url, options = {}) {
    if (DEMO_MODE) {
        return { ok: false, demo: true, status: 503 };
    }

    try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 10000); // 10s timeout

        const response = await fetch(url, {
            ...options,
            signal: controller.signal
        });

        clearTimeout(timeoutId);
        return response;
    } catch (error) {
        console.warn('API unavailable:', error.message);
        return { ok: false, error: error.message, status: 0 };
    }
}
