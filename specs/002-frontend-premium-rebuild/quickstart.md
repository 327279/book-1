# Quickstart Guide: Premium Frontend Rebuild

**Feature**: `002-frontend-premium-rebuild`
**Created**: 2025-12-18

## Prerequisites

- Node.js 18+ installed
- npm or yarn package manager
- Git for version control
- Code editor (VS Code recommended)

## Local Development Setup

### 1. Navigate to Frontend Directory

```bash
cd c:\Users\ARComputers\OneDrive\Desktop\book\frontend
```

### 2. Install Dependencies

```bash
npm install
```

This installs all required packages including Docusaurus and React.

### 3. Start Development Server

```bash
npm start
```

The site will open at `http://localhost:3000` with hot-reload enabled.

### 4. Development Workflow

#### Making CSS Changes

**For Design System (Shared Tokens)**:
- Edit: `frontend/src/css/custom.css`
- Changes apply globally across all components

**For Component-Specific Styles**:
- Edit: `frontend/src/components/[ComponentName]/[ComponentName].module.css`
- Changes are scoped to that component only

#### Testing Responsive Design

**Using Browser DevTools**:
1. Open DevTools (F12)
2. Click device toolbar icon (Ctrl+Shift+M)
3. Test viewports:
   - Mobile: 375px
   - Tablet: 768px
   - Desktop: 1920px

#### Testing Dark Mode

1. Click the moon/sun icon in navbar
2. Verify all components switch themes
3. Check contrast with browser DevTools

### 5. Build for Production

```bash
npm run build
```

Output will be in `frontend/build/` directory.

### 6. Preview Production Build

```bash
npm run serve
```

## File Structure for This Feature

```
frontend/src/
├── css/
│   └── custom.css                      # Design system tokens
├── components/
│   ├── ChatBot/
│   │   ├── ChatBot.jsx
│   │   ├── ChatInterface.jsx
│   │   ├── MessageRenderer.jsx
│   │   ├── ChatBot.module.css          # Component styles
│   │   └── index.jsx                   # Exports
│   ├── TextSelector/
│   │   ├── TextSelector.jsx
│   │   ├── TextSelector.module.css     # Component styles
│   │   └── index.jsx                   # Exports
│   └── Auth/
│       ├── SignInForm.js
│       ├── SignupForm.js
│       ├── AuthContext.js
│       ├── authClient.js
│       ├── styles.module.css           # Component styles
│       └── index.js                    # Exports
├── pages/
│   ├── index.js                        # Landing page
│   └── index.module.css                # Landing page styles
└── theme/
    └── Layout/
        ├── index.js
        └── styles.module.css
```

## Development Tools

### Recommended VS Code Extensions

- **ESLint**: JavaScript linting
- **Prettier**: Code formatting
- **CSS Intellisense**: CSS class autocomplete
- **Auto Rename Tag**: HTML/JSX tag renaming

### Browser Extensions

- **React Developer Tools**: Component debugging
- **axe DevTools**: Accessibility testing

## Testing Checklist

Before submitting changes, verify:

- [ ] `npm run build` completes without errors
- [ ] No CSS import warnings
- [ ] Landing page displays correctly
- [ ] Chat bot opens and functions
- [ ] Dark mode works on all components
- [ ] Mobile layout works (resize to 375px)
- [ ] All interactive elements have hover states
- [ ] Focus states are visible for accessibility

## Common Tasks

### Adding a New CSS Variable

1. Open `frontend/src/css/custom.css`
2. Add to appropriate section under `:root`
3. Add dark mode variant under `[data-theme='dark']` if needed
4. Use in components: `var(--your-variable-name)`

### Creating a New Component CSS Module

1. Create `ComponentName.module.css` in component folder
2. Define classes: `.className { }`
3. Import in JSX: `import styles from './ComponentName.module.css'`
4. Use in JSX: `className={styles.className}`

### Debugging CSS Issues

1. Open Browser DevTools (F12)
2. Inspect element
3. Check computed styles
4. Verify CSS variables are resolving
5. Check for specificity conflicts

## Performance Monitoring

### Using Lighthouse

1. Open Chrome DevTools
2. Go to "Lighthouse" tab
3. Run audit for:
   - Performance
   - Accessibility
   - Best Practices

### Target Scores
- Performance: 90+
- Accessibility: 100
- Best Practices: 90+

## Getting Help

### Documentation
- Docusaurus: https://docusaurus.io/docs
- CSS Modules: https://github.com/css-modules/css-modules
- React: https://react.dev

### Spec Kit Plus Framework

All planning documents are in:
```
specs/002-frontend-premium-rebuild/
├── spec.md          # Feature specification
├── plan.md          # Implementation plan
├── data-model.md    # CSS architecture
├── tasks.md         # Task breakdown
└── quickstart.md    # This file
```
