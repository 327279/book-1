import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

const features = [
    {
        icon: '🤖',
        title: 'ROS 2 Fundamentals',
        description: 'Master the Robot Operating System 2, the industry standard middleware powering modern robotics.',
        link: '/docs/chapter-1-ros2'
    },
    {
        icon: '🎮',
        title: 'Simulation & Digital Twins',
        description: 'Build and test in Gazebo and NVIDIA Isaac Sim before deploying to real hardware.',
        link: '/docs/chapter-5-gazebo'
    },
    {
        icon: '🧠',
        title: 'Vision-Language-Action',
        description: 'Connect LLMs to physical robots using VLA models and natural language commands.',
        link: '/docs/chapter-13-vla'
    },
    {
        icon: '🦾',
        title: 'Humanoid Robotics',
        description: 'Explore locomotion, manipulation, and the future of human-robot interaction.',
        link: '/docs/chapter-14-humanoids'
    }
];

function HeroSection() {
    const { siteConfig } = useDocusaurusContext();
    return (
        <div className={styles.hero}>
            <div className={styles.heroContent}>
                <span className={styles.badge}>Open Source Textbook</span>
                <h1 className={styles.heroTitle}>
                    Physical AI &<br />Humanoid Robotics
                </h1>
                <p className={styles.heroSubtitle}>
                    A comprehensive guide to building intelligent robots with ROS 2,
                    simulation, and modern AI. From theory to deployment.
                </p>
                <div className={styles.heroButtons}>
                    <Link className={styles.primaryButton} to="/docs/chapter-1-ros2">
                        Start Learning
                    </Link>
                    <Link className={styles.secondaryButton} to="/docs/intro">
                        View Curriculum
                    </Link>
                </div>
            </div>
        </div>
    );
}

function FeatureCard({ icon, title, description, link }) {
    return (
        <Link to={link} className={styles.featureCard}>
            <span className={styles.featureIcon}>{icon}</span>
            <h3 className={styles.featureTitle}>{title}</h3>
            <p className={styles.featureDescription}>{description}</p>
            <span className={styles.featureLink}>Learn more →</span>
        </Link>
    );
}

function FeaturesSection() {
    return (
        <section className={styles.features}>
            <div className={styles.container}>
                <h2 className={styles.sectionTitle}>What You'll Learn</h2>
                <div className={styles.featureGrid}>
                    {features.map((feature, idx) => (
                        <FeatureCard key={idx} {...feature} />
                    ))}
                </div>
            </div>
        </section>
    );
}

function StatsSection() {
    return (
        <section className={styles.stats}>
            <div className={styles.container}>
                <div className={styles.statItem}>
                    <span className={styles.statNumber}>18+</span>
                    <span className={styles.statLabel}>Chapters</span>
                </div>
                <div className={styles.statItem}>
                    <span className={styles.statNumber}>100+</span>
                    <span className={styles.statLabel}>Code Examples</span>
                </div>
                <div className={styles.statItem}>
                    <span className={styles.statNumber}>AI</span>
                    <span className={styles.statLabel}>Powered Assistant</span>
                </div>
            </div>
        </section>
    );
}

export default function Home() {
    const { siteConfig } = useDocusaurusContext();
    return (
        <Layout
            title="Physical AI & Humanoid Robotics"
            description="Interactive textbook for learning robotics with ROS 2, simulation, and AI">
            <HeroSection />
            <StatsSection />
            <FeaturesSection />
        </Layout>
    );
}
