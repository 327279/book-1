// @ts-check
// `@type` JSDoc annotations allow TypeScript to help with type checking
// and IDEs autocompletion

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Advanced Textbook for Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-username', // Usually your GitHub org/user name.
  projectName: 'book', // Usually your repo name.

  onBrokenLinks: 'warn',  // Change from 'throw' to 'warn' to allow build to continue
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Logo',
          src: 'img/logo.png',
          srcDark: 'img/logo.png', // If you have a dark mode logo
          width: 32,
          height: 32,
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Chapters',
          },
          {
            type: 'dropdown',
            label: 'Resources',
            position: 'left',
            items: [
              {
                label: 'GitHub Repository',
                href: 'https://github.com/your-username/book',
              },
              {
                label: 'Research Papers',
                href: 'https://example.com/papers',
              },
              {
                label: 'Video Lectures',
                href: 'https://example.com/lectures',
              },
            ],
          },
          {
            href: 'https://github.com/your-username/book',
            position: 'right',
            className: 'navbar-github-link',
            'aria-label': 'GitHub repository',
          },
          {
            to: '/signup',
            label: 'Join Class 🚀',
            position: 'right',
            className: 'button button--primary button--sm margin-left--sm',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Academic Content',
            items: [
              {
                label: 'Chapter 1: ROS 2',
                to: '/docs/chapter-1-ros2',
              },
              {
                label: 'Chapter 2: Simulation',
                to: '/docs/chapter-2-simulation',
              },
              {
                label: 'Chapter 3: NVIDIA Isaac',
                to: '/docs/chapter-3-isaac',
              },
              {
                label: 'Chapter 4: VLA',
                to: '/docs/chapter-4-vla',
              },
              {
                label: 'Chapter 5: Capstone',
                to: '/docs/chapter-5-capstone',
              },
              {
                label: 'Chapter 6: RAG Implementation',
                to: '/docs/chapter-6-rag-implementation',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-username/book',
              },
              {
                label: 'Research Papers',
                href: 'https://example.com/papers',
              },
              {
                label: 'Video Lectures',
                href: 'https://example.com/lectures',
              },
            ],
          },
          {
            title: 'Legal',
            items: [
              {
                label: 'Privacy Policy',
                href: 'https://example.com/privacy',
              },
              {
                label: 'Terms of Use',
                href: 'https://example.com/terms',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. All rights reserved.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.vsDark,
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      docs: {
        sidebar: {
          hideable: true,
          autoCollapseCategories: true,
        },
      },
    }),
};

module.exports = config;