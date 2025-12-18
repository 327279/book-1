/**
 * API Configuration
 * Backend URL configuration for Physical AI & Humanoid Robotics textbook
 */

// Backend API base URL
// Priority: 1) Runtime config, 2) Docusaurus config, 3) localhost fallback
export const API_BASE_URL = (() => {
    if (typeof window !== 'undefined') {
        // Check for runtime config first
        if (window.__ENV__?.API_URL) return window.__ENV__.API_URL;
        // Check for Docusaurus site config
        if (window.docusaurusConfig?.customFields?.apiUrl) return window.docusaurusConfig.customFields.apiUrl;
    }
    // Fallback to localhost for development
    return 'http://localhost:8000';
})();

// Check if we're in production
export const IS_PRODUCTION = typeof window !== 'undefined' &&
    window.location.hostname !== 'localhost';

// API endpoints
export const API_ENDPOINTS = {
    queries: `${API_BASE_URL}/api/v1/queries/`,
    agentSkill: `${API_BASE_URL}/api/v1/agent/skill`,
    userProfile: `${API_BASE_URL}/api/v1/users/profile`,
    auth: `${API_BASE_URL}/api/auth`,
};

// For backward compatibility - always try real API
export const DEMO_MODE = false;

/**
 * Safe fetch wrapper that handles backend unavailability gracefully
 * Always attempts the real API call, returns error response if fails
 */
export async function safeFetch(url, options = {}) {
    try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 15000); // 15s timeout for serverless cold starts

        const response = await fetch(url, {
            ...options,
            signal: controller.signal

        });

        clearTimeout(timeoutId);
        return response;
    } catch (error) {
        console.warn('API call failed:', error.message);
        return { ok: false, error: error.message, status: 0 };
    }
}

