/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      items: [
        'chapter-0-intro',
        'chapter-0-5-setup',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'chapter-1-ros2',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Simulation)',
      items: [
        'chapter-2-simulation',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'chapter-3-isaac',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'chapter-4-vla',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Humanoid Development',
      items: [
        'chapter-5-humanoids',
        'chapter-5-capstone',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Conversational Robotics & RAG',
      items: [
        'chapter-6-conversational',
        'chapter-6-rag-implementation',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Advanced Concepts',
      items: [
        'chapter-7-why-physical-ai-matters',
        'chapter-8-architecture-core-concepts',
        'chapter-9-advanced-topics-physical-ai',
        'chapter-10-conversational-robotics-assessments',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Hardware and Deployment',
      items: [
        'chapter-11-hardware-requirements-lab-setup',
        'chapter-12-cloud-onpremise-deployment',
        'chapter-13-economy-jetson-student-kit',
        'chapter-14-future',
      ],
      collapsed: false,
    },
  ],
};

module.exports = sidebars;