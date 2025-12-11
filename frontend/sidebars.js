/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'chapter-1-ros2',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Simulation and Digital Twins',
      items: [
        'chapter-2-simulation',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'AI-Robot Integration',
      items: [
        'chapter-3-isaac',
        'chapter-4-vla',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Capstone Projects',
      items: [
        'chapter-5-capstone',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'AI Integration and Tools',
      items: [
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
      ],
      collapsed: false,
    },
  ],
};

module.exports = sidebars;