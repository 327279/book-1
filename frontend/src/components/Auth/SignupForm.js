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
        <div className="card margin-vert--lg padding--lg">
            <h2>🚀 Join the Physical AI Class</h2>
            <p>Sign up to personalize your learning experience. We'll ask about your background to customize content for you.</p>

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
                        placeholder="At least 8 characters"
                        required
                        value={formData.password}
                        onChange={e => setFormData({ ...formData, password: e.target.value })}
                    />
                </div>

                <div className="margin-bottom--md">
                    <label><strong>Confirm Password</strong></label>
                    <input
                        className="button button--block button--outline button--secondary"
                        type="password"
                        placeholder="Confirm your password"
                        required
                        value={formData.confirmPassword}
                        onChange={e => setFormData({ ...formData, confirmPassword: e.target.value })}
                    />
                </div>

                <hr className="margin-vert--md" />
                <h4>📊 Tell us about your background</h4>
                <p className="text--secondary">This helps us personalize chapter content for your skill level.</p>

                <div className="margin-bottom--md">
                    <label><strong>Software Background</strong></label>
                    <select
                        className="button button--block button--outline button--secondary"
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

                <div className="margin-bottom--md">
                    <label><strong>Hardware / Robotics Background</strong></label>
                    <select
                        className="button button--block button--outline button--secondary"
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
                    className="button button--primary button--block"
                    disabled={loading}
                >
                    {loading ? 'Creating Account...' : '🎓 Start Learning'}
                </button>

                <p className="text--center margin-top--md">
                    Already have an account? <a href="/signin">Sign In</a>
                </p>
            </form>
        </div>
    );
}
