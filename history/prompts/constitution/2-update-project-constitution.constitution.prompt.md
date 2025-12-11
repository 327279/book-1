---
id: 2
title: "Update Project Constitution"
stage: constitution
date_iso: "2025-12-10"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "master"
user: "ARComputers"
command: "/sp.constitution Project: Physical AI & Humanoid Robotics..."
labels: ["constitution", "update", "project-revision"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - .specify/memory/constitution.md
tests_yaml: |
  - none
prompt_text: |
  sp.constitution

  Project: Physical AI & Humanoid Robotics: Interactive Docusaurus Book & RAG Agent

  Core principles:
  - Spec-Driven Development (Strict adherence to Spec-Kit Plus workflow)
  - AI-First Authorship (Primary content generation via Claude Code)
  - Interactive Pedagogy (Learning through reading AND active querying)
  - Embodied Intelligence (Focus on bridging digital AI with physical robotics)
  - Modular Architecture (Decoupling book content from chatbot backend)

  Key standards:
  - Documentation Framework: Docusaurus (React-based static site generator)
  - Content Curriculum:
      - Module 1: ROS 2 & rclpy (Robotic Nervous System)
      - Module 2: Gazebo & Unity (Digital Twin/Simulation)
      - Module 3: NVIDIA Isaac Sim & Nav2 (AI-Robot Brain)
      - Module 4: VLA & OpenAI Whisper (Vision-Language-Action)
  - RAG Tech Stack:
      - Frontend: OpenAI Agents / ChatKit SDKs
      - Backend: FastAPI (Python)
      - Database: Neon Serverless Postgres
      - Vector Store: Qdrant Cloud (Free Tier)
  - Deployment: GitHub Pages (Book) + Cloud Hosting (API)

  Constraints:
  - Tooling: Must utilize Claude Code CLI and Spec-Kit Plus structure
  - Integration: Chatbot must be embedded directly into Docusaurus pages
  - Context Window: Bot must support answering questions based on user-selected text
  - Cost Efficiency: Architecture must utilize free-tier resources (Qdrant/Neon) where possible
  - Capstone: Must include "The Autonomous Humanoid" final project specifications

  Success criteria:
  - Live Book: Docusaurus site successfully deployed to GitHub Pages
  - Functional RAG: Chatbot accurately answers questions derived from book content
  - Feature verification: User can select text on a page and query the bot specifically about that selection
  - Curriculum Completeness: All 4 Modules and Capstone fully written and coded
response_text: |
  Updated the project constitution file at .specify/memory/constitution.md with 5 core principles:
  1. Spec-Driven Development
  2. AI-First Authorship
  3. Interactive Pedagogy
  4. Embodied Intelligence
  5. Modular Architecture

  The constitution now includes detailed sections for Documentation Framework, Content Curriculum (4 modules + capstone), RAG Tech Stack, Deployment Strategy, Constraints, and Success Criteria that align with the Physical AI & Humanoid Robotics project requirements.
outcome: "Successfully updated the project constitution based on user requirements"
evaluation: "The constitution accurately reflects the project mission, curriculum, technical stack, and success criteria provided by the user"
---

# Update Project Constitution

## Summary
Updated the foundational constitution document for the Physical AI & Humanoid Robotics project, revising core principles and adding detailed specifications for curriculum, technology stack, and success criteria.

## Details
The constitution has been revised to establish five core principles that align with the project's focus on creating an interactive educational experience for humanoid robotics. The document now includes specific sections for the documentation framework (Docusaurus), detailed content curriculum covering ROS 2, simulation environments, AI-robot interfaces, and vision-language-action systems. The RAG tech stack, deployment strategy, constraints, and success criteria are all clearly defined.

## Acceptance Criteria
- [x] Constitution file updated at .specify/memory/constitution.md
- [x] Five core principles established reflecting project requirements
- [x] Content curriculum section includes all 4 modules and capstone
- [x] RAG tech stack details all components (frontend, backend, database, vector store)
- [x] Constraints section addresses all specified requirements
- [x] Success criteria align with project goals