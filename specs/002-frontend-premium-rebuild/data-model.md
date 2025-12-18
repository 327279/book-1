# CSS Architecture & Design System - Premium Frontend Rebuild

**Feature**: `002-frontend-premium-rebuild`
**Created**: 2025-12-18

## Design Tokens

### Color System

#### Light Mode
```css
--primary-50: #eff6ff;
--primary-100: #dbeafe;
--primary-200: #bfdbfe;
--primary-300: #93c5fd;
--primary-400: #60a5fa;
--primary-500: #3b82f6;   /* Primary brand color */
--primary-600: #2563eb;
--primary-700: #1d4ed8;
--primary-800: #1e40af;
--primary-900: #1e3a8a;

--gray-50: #f8fafc;
--gray-100: #f1f5f9;
--gray-200: #e2e8f0;
--gray-300: #cbd5e1;
--gray-400: #94a3b8;
--gray-500: #64748b;
--gray-600: #475569;
--gray-700: #334155;
--gray-800: #1e293b;
--gray-900: #0f172a;

--accent-teal: #14b8a6;
--accent-orange: #f97316;
--accent-green: #22c55e;
--accent-red: #ef4444;
```

#### Dark Mode
```css
[data-theme='dark'] {
  --primary-400: #60a5fa;
  --primary-500: #3b82f6;
  --primary-600: #2563eb;
  
  --bg-primary: #0f172a;
  --bg-secondary: #1e293b;
  --bg-tertiary: #334155;
  
  --text-primary: #f1f5f9;
  --text-secondary: #cbd5e1;
  --text-muted: #94a3b8;
}
```

### Typography

```css
--font-sans: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', system-ui, sans-serif;
--font-mono: 'JetBrains Mono', 'Fira Code', 'Consolas', 'Monaco', monospace;

--text-xs: 0.75rem;      /* 12px */
--text-sm: 0.875rem;     /* 14px */
--text-base: 1rem;       /* 16px */
--text-lg: 1.125rem;     /* 18px */
--text-xl: 1.25rem;      /* 20px */
--text-2xl: 1.5rem;      /* 24px */
--text-3xl: 1.875rem;    /* 30px */
--text-4xl: 2.25rem;     /* 36px */
--text-5xl: 3rem;        /* 48px */

--font-normal: 400;
--font-medium: 500;
--font-semibold: 600;
--font-bold: 700;
--font-extrabold: 800;
```

### Spacing

```css
--space-1: 0.25rem;   /* 4px */
--space-2: 0.5rem;    /* 8px */
--space-3: 0.75rem;   /* 12px */
--space-4: 1rem;      /* 16px */
--space-5: 1.25rem;   /* 20px */
--space-6: 1.5rem;    /* 24px */
--space-8: 2rem;      /* 32px */
--space-10: 2.5rem;   /* 40px */
--space-12: 3rem;     /* 48px */
--space-16: 4rem;     /* 64px */
--space-20: 5rem;     /* 80px */
```

### Border Radius

```css
--radius-sm: 4px;
--radius-md: 8px;
--radius-lg: 12px;
--radius-xl: 16px;
--radius-2xl: 24px;
--radius-full: 9999px;
```

### Shadows

```css
--shadow-sm: 0 1px 2px 0 rgb(0 0 0 / 0.05);
--shadow-md: 0 4px 6px -1px rgb(0 0 0 / 0.1), 0 2px 4px -2px rgb(0 0 0 / 0.1);
--shadow-lg: 0 10px 15px -3px rgb(0 0 0 / 0.1), 0 4px 6px -4px rgb(0 0 0 / 0.1);
--shadow-xl: 0 20px 25px -5px rgb(0 0 0 / 0.1), 0 8px 10px -6px rgb(0 0 0 / 0.1);
--shadow-2xl: 0 25px 50px -12px rgb(0 0 0 / 0.25);
```

### Animations

```css
--transition-fast: 150ms cubic-bezier(0.4, 0, 0.2, 1);
--transition-base: 200ms cubic-bezier(0.4, 0, 0.2, 1);
--transition-slow: 300ms cubic-bezier(0.4, 0, 0.2, 1);

--easing-linear: linear;
--easing-in: cubic-bezier(0.4, 0, 1, 1);
--easing-out: cubic-bezier(0, 0, 0.2, 1);
--easing-in-out: cubic-bezier(0.4, 0, 0.2, 1);
```

## Component CSS Modules

### ChatBot.module.css

**Purpose**: Scoped styles for chat interface components

**Key Classes**:
- `.fab` - Floating action button with pulse animation
- `.window` - Chat window container with glassmorphism
- `.header` - Chat header with gradient background
- `.messages` - Scrollable message area
- `.message` - Individual message bubble
- `.userMessage` - User-specific message styling
- `.botMessage` - Bot-specific message styling
- `.form` - Input form container
- `.input` - Text input with focus states
- `.sendButton` - Send button with hover effects

### TextSelector.module.css

**Purpose**: Scoped styles for text selection indicator

**Key Classes**:
- `.indicator` - Selection indicator container
- `.tooltip` - Tooltip-style preview box
- `.selectedText` - Truncated text preview
- `.fadeIn` - Fade-in animation
- `.fadeOut` - Fade-out animation

### Auth/styles.module.css

**Purpose**: Scoped styles for authentication forms

**Key Classes**:
- `.modal` - Modal overlay and container
- `.card` - Auth card with elevation
- `.form` - Form layout container
- `.fieldGroup` - Input field grouping
- `.label` - Form label styling
- `.input` - Input field with focus states
- `.error` - Error message styling
- `.success` - Success message styling
- `.button` - Primary action button
- `.secondaryButton` - Secondary action button
- `.link` - Text link styling

### index.module.css (Landing Page)

**Purpose**: Scoped styles for homepage sections

**Key Classes**:
- `.hero` - Hero section with gradient
- `.heroContent` - Content container
- `.heroTitle` - Large heading
- `.heroSubtitle` - Subtitle text
- `.badge` - Pill badge component
- `.primaryButton` - Primary CTA button
- `.secondaryButton` - Secondary CTA button
- `.stats` - Statistics section
- `.statItem` - Individual stat display
- `.features` - Features grid container
- `.featureCard` - Feature card with hover
- `.featureIcon` - Icon placeholder
- `.featureTitle` - Feature heading
- `.featureDescription` - Feature description

## Responsive Breakpoints

```css
/* Mobile first approach */
@media (min-width: 640px)  { /* sm */ }
@media (min-width: 768px)  { /* md */ }
@media (min-width: 1024px) { /* lg */ }
@media (min-width: 1280px) { /* xl */ }
@media (min-width: 1536px) { /* 2xl */ }
```

## Accessibility Standards

### Color Contrast
- Text on background: Minimum 4.5:1 (WCAG AA)
- Large text (18pt+): Minimum 3:1
- Interactive elements: Minimum 3:1

### Focus States
- All interactive elements must have visible focus indicator
- Focus ring: `outline: 2px solid var(--primary-500); outline-offset: 2px;`

### Motion
- Respect `prefers-reduced-motion` media query
- Disable animations if user prefers reduced motion

## CSS Module Naming Convention

```css
/* Component-specific styles */
.componentName { }
.componentName__element { }
.componentName__element--modifier { }

/* State classes */
.is-active { }
.is-disabled { }
.is-loading { }
```

## Z-Index Scale

```css
--z-base: 0;
--z-dropdown: 1000;
--z-sticky: 1020;
--z-fixed: 1030;
--z-modal-backdrop: 1040;
--z-modal: 1050;
--z-popover: 1060;
--z-tooltip: 1070;
```

## File Organization

```
frontend/src/css/
└── custom.css           # Global design system

frontend/src/components/
├── ChatBot/
│   └── ChatBot.module.css
├── TextSelector/
│   └── TextSelector.module.css
└── Auth/
    └── styles.module.css

frontend/src/pages/
└── index.module.css

frontend/src/theme/
├── Layout/
│   └── styles.module.css
└── DocItem/Content/
    └── styles.module.css
```
