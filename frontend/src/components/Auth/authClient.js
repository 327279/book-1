import { createAuthClient } from "better-auth/react";
import { API_BASE_URL, DEMO_MODE } from '../../config/api';

// Create the Better-Auth client
export const authClient = createAuthClient({
    baseURL: DEMO_MODE ? "/api/auth" : `${API_BASE_URL}/api/auth`, // Backend auth endpoint
});

// Export auth hooks and utilities
export const {
    signIn,
    signUp,
    signOut,
    useSession,
    getSession
} = authClient;
