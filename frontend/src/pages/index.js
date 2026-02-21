import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { Cpu, Monitor, Brain, Bot, ChevronRight, BookOpen, Code, Sparkles } from 'lucide-react';


import styles from './index.module.css';

const features = [
    {
        icon: <Cpu size={32} />,
        title: 'ROS 2 Fundamentals',
        description: 'Master the Robot Operating System 2, the industry standard middleware powering modern robotics.',
        link: '/docs/chapter-1-ros2'
    },
    {
        icon: <Monitor size={32} />,
        title: 'Simulation & Digital Twins',
        description: 'Build and test in Gazebo and NVIDIA Isaac Sim before deploying to real hardware.',
        link: '/docs/chapter-5-gazebo'
    },
    {
        icon: <Brain size={32} />,
        title: 'Vision-Language-Action',
        description: 'Connect LLMs to physical robots using VLA models and natural language commands.',
        link: '/docs/chapter-13-vla'
    },
    {
        icon: <Bot size={32} />,
        title: 'Humanoid Robotics',
        description: 'Explore locomotion, manipulation, and the future of human-robot interaction.',
        link: '/docs/chapter-14-humanoids'
    }
];

function HeroSection() {
    const { siteConfig } = useDocusaurusContext();
    return (
        <div className={styles.hero}>
            <div className={styles.heroBackground}>
                <div className={styles.glowBlob}></div>
                <div className={styles.meshGrid}></div>
            </div>
            <div className={styles.container}>
                <div className={styles.heroContent}>
                    <div className={styles.badgeContainer}>
                        <span className={styles.badge}>
                            <Sparkles size={14} className={styles.badgeIcon} />
                            Physical AI Textbook 2026
                        </span>
                    </div>
                    <h1 className={styles.heroTitle}>
                        The Future of<br />
                        <span>Humanoid Robotics</span>
                    </h1>
                    <p className={styles.heroSubtitle}>
                        A comprehensive, interactive guide to building intelligent embodied agents.
                        Master ROS 2, Simulation, and VLA Models from the ground up.
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
                <div className={styles.heroVisual}>
                    <div className={styles.visualMain}>
                        <div className={styles.orbInner}></div>
                        <div className={styles.orbOuter}></div>
                        <Cpu className={styles.visualIcon} size={80} />
                    </div>
                    <div className={styles.visualDecorative1}></div>
                    <div className={styles.visualDecorative2}></div>
                </div>
            </div>
        </div>
    );
}

function FeatureCard({ icon, title, description, link }) {
    return (
        <Link to={link} className={styles.featureCard}>
            <div className={styles.featureIconContainer}>
                {icon}
            </div>
            <h3 className={styles.featureTitle}>{title}</h3>
            <p className={styles.featureDescription}>{description}</p>
            <span className={styles.featureLink}>
                Learn more <ChevronRight size={16} />
            </span>
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
                    <div className={styles.statIcon}><BookOpen size={24} /></div>
                    <div>
                        <span className={styles.statNumber}>18+</span>
                        <span className={styles.statLabel}>Chapters</span>
                    </div>
                </div>
                <div className={styles.statItem}>
                    <div className={styles.statIcon}><Code size={24} /></div>
                    <div>
                        <span className={styles.statNumber}>100+</span>
                        <span className={styles.statLabel}>Code Examples</span>
                    </div>
                </div>
                <div className={styles.statItem}>
                    <div className={styles.statIcon}><Sparkles size={24} /></div>
                    <div>
                        <span className={styles.statNumber}>AI</span>
                        <span className={styles.statLabel}>Powered Assistant</span>
                    </div>
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
