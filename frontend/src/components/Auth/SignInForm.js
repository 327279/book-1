import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import styles from './styles.module.css';

export default function SignInForm() {
    const { login } = useAuth();
    const [formData, setFormData] = useState({
        email: '',
        password: ''
    });
    const [error, setError] = useState('');
    const [loading, setLoading] = useState(false);

    const handleSubmit = async (e) => {
        e.preventDefault();
        setError('');
        setLoading(true);

        try {
            await login(formData.email, formData.password);
            window.location.href = '/docs/chapter-0-intro';
        } catch (err) {
            setError(err.message || 'Invalid email or password');
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className="card margin-vert--lg padding--lg">
            <h2>👋 Welcome Back</h2>
            <p>Sign in to continue your Physical AI learning journey.</p>

            {error && (
                <div className="alert alert--danger margin-bottom--md">
                    {error}
                </div>
            )}

            <form onSubmit={handleSubmit}>
                <div className="margin-bottom--md">
                    <label><strong>Email</strong></label>
                    <input
                        className="button button--block button--outline button--secondary"
                        type="email"
                        placeholder="your@email.com"
                        required
                        value={formData.email}
                        onChange={e => setFormData({ ...formData, email: e.target.value })}
                    />
                </div>

                <div className="margin-bottom--md">
                    <label><strong>Password</strong></label>
                    <input
                        className="button button--block button--outline button--secondary"
                        type="password"
                        placeholder="Your password"
                        required
                        value={formData.password}
                        onChange={e => setFormData({ ...formData, password: e.target.value })}
                    />
                </div>

                <button
                    type="submit"
                    className="button button--primary button--block"
                    disabled={loading}
                >
                    {loading ? 'Signing In...' : '🔐 Sign In'}
                </button>

                <p className="text--center margin-top--md">
                    Don't have an account? <a href="/signup">Sign Up</a>
                </p>
            </form>
        </div>
    );
}
