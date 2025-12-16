# Project Constitution: Physical AI & Humanoid Robotics

## 1. Vision & Goals

### Core Vision
To create the definitive interactive textbook for "Physical AI & Humanoid Robotics" that bridges the gap between digital AI (LLMs) and embodied intelligence (Robots).

### Learning Philosophy
- **Embodied First**: Theory must lead to simulation or physical implementation.
- **Interactive**: The book talks back via an embedded RAG agent.
- **Spec-Driven**: All work is defined by rigorous specs before code is written.

## 2. Technical Standards

### Architecture
- **Frontend**: Docusaurus 3+ (React)
- **Backend**: FastAPI (Python 3.11+)
- **Data**: Neon (PostgreSQL) + Qdrant (Vectors)
- **Sim**: ROS 2 Humble + NVIDIA Isaac Sim

### Code Quality
- **Typing**: Strict Python type hints.
- **Testing**: Pytest for backend logic (min 80% coverage on core logic).
- **Docs**: READMEs for every module; inline docstrings for complex logic.

## 3. Spec-Driven Workflow (SpecifyKit Plus)
1.  **Phase 0 (Research)**: Analyze syllabus and requirements.
2.  **Phase 1 (Spec)**: Create `spec.md` defining features and success criteria.
3.  **Phase 2 (Plan)**: Create `plan.md` defining architecture.
4.  **Phase 3 (Tasks)**: Create `tasks.md` with granular steps.
5.  **Phase 4 (History)**: Log all prompts in `history/prompts/`.

## 4. Ethical AI
- **Safety**: All robot code must prioritize human safety.
- **Transparency**: AI-generated content in the book must be verified by human experts.
