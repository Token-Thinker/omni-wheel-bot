// Light/dark theme toggle persisted to localStorage. Mirrors the original
// behavior in script.js: default `dark`, attribute on <body>, two buttons.

import { STORAGE_KEYS } from './storage';

export type Theme = 'light' | 'dark';

let current: Theme = 'dark';

export function initTheme(buttons: ReadonlyArray<HTMLElement | null>): void {
  const stored = localStorage.getItem(STORAGE_KEYS.theme);
  applyTheme(stored === 'light' ? 'light' : 'dark');
  for (const btn of buttons) {
    btn?.addEventListener('click', toggleTheme);
  }
}

export function applyTheme(theme: Theme): void {
  current = theme === 'light' ? 'light' : 'dark';
  document.body.setAttribute('data-theme', current);
  localStorage.setItem(STORAGE_KEYS.theme, current);
}

export function toggleTheme(): void {
  applyTheme(current === 'light' ? 'dark' : 'light');
}
