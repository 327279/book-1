import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import styles from './styles.module.css';

export default function SignupForm() {
    const { signup } = useAuth();
    const [formData, setFormData] = useState({
        email: '',
        password: '',
        confirmPassword: '',
        softwareBg: 'Beginner',
        hardwareBg: 'None'
    });
    const [error, setError] = useState('');
    const [loading, setLoading] = useState(false);

    const handleSubmit = async (e) => {
        e.preventDefault();
        setError('');

        if (formData.password !== formData.confirmPassword) {
            setError('Passwords do not match');
            return;
        }

        if (formData.password.length < 8) {
            setError('Password must be at least 8 characters');
            return;
        }

        setLoading(true);
        try {
            await signup(formData.email, formData.password, {
                software_bg: formData.softwareBg,
                hardware_bg: formData.hardwareBg
            });
            alert('Welcome! The book content will now be personalized for you.');
            window.location.href = '/docs/chapter-0-intro';
        } catch (err) {
            setError(err.message || 'Signup failed. Please try again.');
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className={styles.card}>
            <div className={styles.container}>
                <div className={styles.header}>
                    <h2 className={styles.title}>🚀 Join the Physical AI Class</h2>
                    <p className={styles.subtitle}>Sign up to personalize your learning experience.</p>
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
                            placeholder="At least 8 characters"
                            required
                            value={formData.password}
                            onChange={e => setFormData({ ...formData, password: e.target.value })}
                        />
                    </div>

                    <div className={styles.fieldGroup}>
                        <label className={styles.label}>Confirm Password</label>
                        <input
                            className={styles.input}
                            type="password"
                            placeholder="Confirm your password"
                            required
                            value={formData.confirmPassword}
                            onChange={e => setFormData({ ...formData, confirmPassword: e.target.value })}
                        />
                    </div>

                    <div className={styles.divider}>Background Info</div>

                    <div className={styles.fieldGroup}>
                        <label className={styles.label}>Software Background</label>
                        <select
                            className={styles.input}
                            value={formData.softwareBg}
                            onChange={e => setFormData({ ...formData, softwareBg: e.target.value })}
                        >
                            <option value="Beginner">Beginner (No coding experience)</option>
                            <option value="intermediate_python">Intermediate Python Developer</option>
                            <option value="advanced_python">Advanced Python / ML Engineer</option>
                            <option value="advanced_cpp">Advanced C++ Developer</option>
                            <option value="fullstack">Full-Stack Developer</option>
                        </select>
                    </div>

                    <div className={styles.fieldGroup}>
                        <label className={styles.label}>Hardware / Robotics Background</label>
                        <select
                            className={styles.input}
                            value={formData.hardwareBg}
                            onChange={e => setFormData({ ...formData, hardwareBg: e.target.value })}
                        >
                            <option value="None">None - Complete Beginner</option>
                            <option value="arduino_rpi">Arduino / Raspberry Pi Hobbyist</option>
                            <option value="embedded">Embedded Systems Developer</option>
                            <option value="ros_robotics">ROS / Industrial Robotics</option>
                            <option value="mechanical_eng">Mechanical Engineering Background</option>
                        </select>
                    </div>

                    <button
                        type="submit"
                        className={styles.button}
                        disabled={loading}
                    >
                        {loading ? 'Creating Account...' : '🎓 Start Learning'}
                    </button>

                    <div className={styles.linkContainer}>
                        Already have an account? <a className={styles.link} href="/signin">Sign In</a>
                    </div>
                </form>
            </div>
        </div>
    );
}

