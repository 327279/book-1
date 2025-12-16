import React, { createContext, useContext, useState, useEffect } from 'react';

const AuthContext = createContext();

export const AuthProvider = ({ children }) => {
    const [user, setUser] = useState(null);
    const [loading, setLoading] = useState(true);

    useEffect(() => {
        // Check local storage for existing session
        const storedUser = localStorage.getItem('user');
        if (storedUser) {
            setUser(JSON.parse(storedUser));
        }
        setLoading(false);
    }, []);

    const signup = async (email, password, profile) => {
        // Mock Signup - In prod, call Better-Auth API
        const newUser = { email, profile, id: Date.now().toString() };
        setUser(newUser);
        localStorage.setItem('user', JSON.stringify(newUser));
        return newUser;
    };

    const login = async (email, password) => {
        // Mock Login
        const storedUser = JSON.parse(localStorage.getItem('user'));
        if (storedUser && storedUser.email === email) {
            setUser(storedUser);
            return storedUser;
        }
        throw new Error('Invalid credentials');
    };

    const logout = () => {
        setUser(null);
        localStorage.removeItem('user');
    };

    return (
        <AuthContext.Provider value={{ user, signup, login, logout, loading }}>
            {children}
        </AuthContext.Provider>
    );
};

export const useAuth = () => useContext(AuthContext);
