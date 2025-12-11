<!--
Sync Impact Report:
Version change: 1.0.0 → 2.0.0
Modified principles: All principles replaced with new set
Removed sections: Old principles and constraints
Added sections: New core principles, documentation framework, content curriculum, RAG tech stack, deployment strategy, constraints, and success criteria
Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics: Interactive Docusaurus Book & RAG Agent Constitution

## Core Principles

### I. Spec-Driven Development
Strict adherence to Spec-Kit Plus workflow for all development activities. All features, changes, and implementations must be documented in specifications before implementation begins.

### II. AI-First Authorship
Primary content generation via Claude Code with human oversight. All educational content, code examples, and interactive elements originate from AI-assisted development processes.

### III. Interactive Pedagogy
Learning through reading AND active querying. The educational experience combines traditional textbook content with real-time AI interaction capabilities for enhanced comprehension.

### IV. Embodied Intelligence
Focus on bridging digital AI with physical robotics. Content emphasizes the connection between artificial intelligence algorithms and their manifestation in physical robotic systems.

### V. Modular Architecture
Decoupling book content from chatbot backend while maintaining seamless integration. Components must function independently while providing unified user experience.

## Documentation Framework

- Static Site Generator: Docusaurus (React-based)
- Content Format: Markdown with interactive React components
- Navigation: Hierarchical structure supporting 4 modules plus capstone

## Content Curriculum

- Module 1: ROS 2 & rclpy (Robotic Nervous System)
- Module 2: Gazebo & Unity (Digital Twin/Simulation)
- Module 3: NVIDIA Isaac Sim & Nav2 (AI-Robot Brain)
- Module 4: VLA & OpenAI Whisper (Vision-Language-Action)
- Capstone: "The Autonomous Humanoid" final project specifications

## RAG Tech Stack

- Frontend: OpenAI Agents / ChatKit SDKs
- Backend: FastAPI (Python)
- Database: Neon Serverless Postgres
- Vector Store: Qdrant Cloud (Free Tier)

## Deployment Strategy

- Book: GitHub Pages
- API Backend: Cloud Hosting (compatible with FastAPI)
- Integration: Embedded chatbot directly into Docusaurus pages

## Constraints

- Tooling: Must utilize Claude Code CLI and Spec-Kit Plus structure
- Integration: Chatbot must be embedded directly into Docusaurus pages
- Context Window: Bot must support answering questions based on user-selected text
- Cost Efficiency: Architecture must utilize free-tier resources (Qdrant/Neon) where possible
- Capstone: Must include "The Autonomous Humanoid" final project specifications

## Success Criteria

- Live Book: Docusaurus site successfully deployed to GitHub Pages
- Functional RAG: Chatbot accurately answers questions derived from book content
- Feature verification: User can select text on a page and query the bot specifically about that selection
- Curriculum Completeness: All 4 Modules and Capstone fully written and coded

## Governance

This constitution governs all development decisions for the Physical AI & Humanoid Robotics project. All implementations must align with the mission to create an interactive educational experience that bridges digital AI with physical robotics. Changes to core architecture or feature sets require explicit reconsideration of how they support the pedagogical objectives and technical constraints. All code reviews must verify compliance with modular architecture, documentation standards, and cost efficiency requirements.

**Version**: 2.0.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10



