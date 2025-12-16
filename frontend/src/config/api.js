/**
 * API Configuration
 * Uses environment variable for backend URL, falls back to localhost for development
 */

// Backend API base URL
export const API_BASE_URL = typeof window !== 'undefined'
    ? (window.__ENV__?.API_URL || 'http://localhost:8000')
    : 'http://localhost:8000';

// Check if we're in production (Vercel deployment)
export const IS_PRODUCTION = typeof window !== 'undefined' &&
    window.location.hostname !== 'localhost';

// API endpoints
export const API_ENDPOINTS = {
    queries: `${API_BASE_URL}/api/v1/queries/`,
    agentSkill: `${API_BASE_URL}/api/v1/agent/skill`,
    userProfile: `${API_BASE_URL}/api/v1/users/profile`,
    auth: `${API_BASE_URL}/api/auth`,
};

// Demo/Mock mode - enabled when backend is unavailable
export const DEMO_MODE = IS_PRODUCTION;

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
