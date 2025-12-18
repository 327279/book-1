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
        <div className={styles.card}>
            <div className={styles.container}>
                <div className={styles.header}>
                    <h2 className={styles.title}>👋 Welcome Back</h2>
                    <p className={styles.subtitle}>Sign in to continue your Physical AI learning journey.</p>
                </div>

                {error && (
                    <div className={styles.error}>{error}</div>
                )}

                <form className={styles.form} onSubmit={handleSubmit}>
                    <div className={styles.fieldGroup}>
                        <label className={styles.label}>Email</label>
                        <input
                            className={styles.input}
                            type="email"
                            placeholder="your@email.com"
                            required
                            value={formData.email}
                            onChange={e => setFormData({ ...formData, email: e.target.value })}
                        />
                    </div>

                    <div className={styles.fieldGroup}>
                        <label className={styles.label}>Password</label>
                        <input
                            className={styles.input}
                            type="password"
                            placeholder="Your password"
                            required
                            value={formData.password}
                            onChange={e => setFormData({ ...formData, password: e.target.value })}
                        />
                    </div>

                    <button
                        type="submit"
                        className={styles.button}
                        disabled={loading}
                    >
                        {loading ? 'Signing In...' : '🔐 Sign In'}
                    </button>

                    <div className={styles.linkContainer}>
                        Don't have an account? <a className={styles.link} href="/signup">Sign Up</a>
                    </div>
                </form>
            </div>
        </div>
    );
}

