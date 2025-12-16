import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import styles from './styles.module.css';

export default function SignupForm() {
    const { signup } = useAuth();
    const [formData, setFormData] = useState({
        email: '',
        password: '',
        softwareBg: 'Beginner',
        hardwareBg: 'None'
    });

    const handleSubmit = async (e) => {
        e.preventDefault();
        await signup(formData.email, formData.password, {
            software_bg: formData.softwareBg,
            hardware_bg: formData.hardwareBg
        });
        alert('Welcome! The book content will now be personalized for you.');
    };

    return (
        <div className="card margin-vert--lg padding--lg">
            <h2>🚀 Join the Physical AI Class</h2>
            <p>Sign up to personalize your learning experience.</p>
            <form onSubmit={handleSubmit}>
                <div className="margin-bottom--md">
                    <label>Email</label>
                    <input
                        className="button button--block button--outline button--secondary"
                        type="email"
                        required
                        onChange={e => setFormData({ ...formData, email: e.target.value })}
                    />
                </div>
                <div className="margin-bottom--md">
                    <label>Software Background</label>
                    <select
                        className="button button--block button--outline button--secondary"
                        onChange={e => setFormData({ ...formData, softwareBg: e.target.value })}
                    >
                        <option value="Beginner">Beginner (No coding)</option>
                        <option value="intermediate_python">Intermediate Python</option>
                        <option value="advanced_cpp">Advanced C++</option>
                    </select>
                </div>
                <div className="margin-bottom--md">
                    <label>Hardware Background</label>
                    <select
                        className="button button--block button--outline button--secondary"
                        onChange={e => setFormData({ ...formData, hardwareBg: e.target.value })}
                    >
                        <option value="None">None</option>
                        <option value="arduino_rpi">Arduino / Raspberry Pi</option>
                        <option value="ros_robotics">ROS / Industrial Robotics</option>
                    </select>
                </div>
                <button type="submit" className="button button--primary button--block">
                    Start Learning
                </button>
            </form>
        </div>
    );
}
