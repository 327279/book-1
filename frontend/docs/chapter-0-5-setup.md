---
title: "Chapter 0.5 - Setting Up Your Lab"
sidebar_position: 0.5
---

# Setting Up Your Lab

Before we dive into the complex world of robotics, we need to prepare our digital workbench. This chapter will guide you through setting up the essential tools you'll need for this journey.

## 1. The Command Line (Terminal)

Robotics engineers live in the terminal. It is the most direct way to talk to your computer.

### Windows Users
We recommend using **WSL 2 (Windows Subsystem for Linux)**. This gives you a real Ubuntu Linux environment inside Windows.
1. Open PowerShell as Administrator.
2. Run: `wsl --install`
3. Restart your computer.
4. Open the "Ubuntu" app from your Start menu.

### Mac/Linux Users
You are already set! Just open your `Terminal` app.

## 2. Python

Python is the primary language of AI and Robotics.

Check if you have it installed:
```bash
python3 --version
```

If not, download it from [python.org](https://www.python.org) or use your package manager (`apt install python3` on Ubuntu/WSL).

## 3. Visual Studio Code (VS Code)

This is the code editor we will use. It has amazing extensions for Python and ROS.
1. Download and install [VS Code](https://code.visualstudio.com/).
2. Install the **"Remote - WSL"** extension if you are on Windows.
3. Install the **"Python"** extension.

## 4. Git

Git helps us save our work and collaborate.
```bash
git --version
```
If you don't have it:
- **Ubuntu/WSL**: `sudo apt install git`
- **Mac**: `brew install git`

## Ready?

Once you have your terminal, Python, and VS Code ready, you are prepared to tackle the "Nervous System" of robotics in the next chapter.

> [!TIP]
> Don't worry if the terminal feels scary at first. We will explain every command we use.

[Proceed to Chapter 1: ROS 2](./chapter-1-ros2.md)
