import { createAuthClient } from "better-auth/react";

// Create the Better-Auth client
export const authClient = createAuthClient({
    baseURL: "http://localhost:8000/api/auth", // Backend auth endpoint
});

// Export auth hooks and utilities
export const {
    signIn,
    signUp,
    signOut,
    useSession,
    getSession
} = authClient;
