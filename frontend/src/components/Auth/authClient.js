import { createAuthClient } from "better-auth/react";
import { API_BASE_URL } from '../../config/api';

// Create the Better-Auth client - always use the configured API base URL
export const authClient = createAuthClient({
    baseURL: `${API_BASE_URL}/api/auth`, // Backend auth endpoint
});

// Export auth hooks and utilities
export const {
    signIn,
    signUp,
    signOut,
    useSession,
    getSession
} = authClient;
