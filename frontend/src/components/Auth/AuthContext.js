import React, { createContext, useContext, useState, useEffect } from 'react';
import { authClient, useSession } from './authClient';
import { API_ENDPOINTS, DEMO_MODE, safeFetch } from '../../config/api';

const AuthContext = createContext();

export const AuthProvider = ({ children }) => {
    const [user, setUser] = useState(null);
    const [loading, setLoading] = useState(true);

    useEffect(() => {
        // Check for existing session using Better-Auth
        const checkSession = async () => {
            try {
                const session = await authClient.getSession();
                if (session?.user) {
                    // Get user profile from localStorage (stored during signup)
                    const storedProfile = localStorage.getItem('userProfile');
                    const profile = storedProfile ? JSON.parse(storedProfile) : {
                        software_bg: 'General',
                        hardware_bg: 'General'
                    };
                    setUser({
                        ...session.user,
                        profile
                    });
                }
            } catch (error) {
                console.log('No active session');
            } finally {
                setLoading(false);
            }
        };
        checkSession();
    }, []);

    const signup = async (email, password, profile) => {
        try {
            const result = await authClient.signUp.email({
                email,
                password,
                name: email.split('@')[0], // Use email prefix as name
            });

            if (result.user) {
                // Store profile in localStorage for now (can be moved to DB with custom fields)
                localStorage.setItem('userProfile', JSON.stringify(profile));

                // Only sync with backend if not in demo mode
                if (!DEMO_MODE) {
                    try {
                        await safeFetch(API_ENDPOINTS.userProfile, {
                            method: 'POST',
                            headers: { 'Content-Type': 'application/json' },
                            body: JSON.stringify({
                                user_id: result.user.id,
                                email: email,
                                software_bg: profile.software_bg,
                                hardware_bg: profile.hardware_bg
                            })
                        });
                    } catch (e) {
                        console.log('Backend profile sync skipped');
                    }
                }

                setUser({
                    ...result.user,
                    profile
                });
                return result.user;
            }
        } catch (error) {
            console.error('Signup error:', error);
            throw error;
        }
    };

    const login = async (email, password) => {
        try {
            const result = await authClient.signIn.email({
                email,
                password,
            });

            if (result.user) {
                // Retrieve stored profile
                const storedProfile = localStorage.getItem('userProfile');
                const profile = storedProfile ? JSON.parse(storedProfile) : {
                    software_bg: 'General',
                    hardware_bg: 'General'
                };

                setUser({
                    ...result.user,
                    profile
                });
                return result.user;
            }
        } catch (error) {
            console.error('Login error:', error);
            throw error;
        }
    };

    const logout = async () => {
        try {
            await authClient.signOut();
            setUser(null);
            localStorage.removeItem('userProfile');
        } catch (error) {
            console.error('Logout error:', error);
        }
    };

    return (
        <AuthContext.Provider value={{ user, signup, login, logout, loading }}>
            {children}
        </AuthContext.Provider>
    );
};

export const useAuth = () => useContext(AuthContext);
