import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
    const { siteConfig } = useDocusaurusContext();
    return (
        <header className={clsx('hero hero--primary', styles.heroBanner)}>
            <div className="container">
                <Heading as="h1" className="hero__title">
                    {siteConfig.title}
                </Heading>
                <p className="hero__subtitle">{siteConfig.tagline}</p>
                <div className={styles.buttons}>
                    <Link
                        className="button button--secondary button--lg"
                        to="/docs/chapter-1-ros2">
                        Start Learning - 5min ⏱️
                    </Link>
                </div>
            </div>
        </header>
    );
}

function Feature({ title, description }) {
    return (
        <div className={clsx('col col--4')}>
            <div className="text--center padding-horiz--md">
                <Heading as="h3">{title}</Heading>
                <p>{description}</p>
            </div>
        </div>
    );
}

export default function Home() {
    const { siteConfig } = useDocusaurusContext();
    return (
        <Layout
            title={`Hello from ${siteConfig.title}`}
            description="Interactive Textbook for Physical AI & Humanoid Robotics">
            <HomepageHeader />
            <main>
                <section className={styles.features}>
                    <div className="container">
                        <div className="row">
                            <Feature
                                title="Embodied Intelligence"
                                description="Move beyond chatbots. Learn to control physical agents using ROS 2, the industry standard middleware for robotics."
                            />
                            <Feature
                                title="Simulation First"
                                description="Master the 'Digital Twin' workflow. Train agents in NVIDIA Isaac Sim™ and Gazebo before deploying to real hardware."
                            />
                            <Feature
                                title="Interactive RAG"
                                description="This book talks back. Use the built-in AI agent to query content, summarize chapters, and translate complex concepts."
                            />
                        </div>
                    </div>
                </section>
            </main>
        </Layout>
    );
}
