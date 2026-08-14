// Single source of truth for localStorage keys used across modules.

const STORAGE_PREFIX = 'owb_ui_';

export const STORAGE_KEYS = {
  theme: `${STORAGE_PREFIX}theme`,
  curve: `${STORAGE_PREFIX}curve`,
  trim: `${STORAGE_PREFIX}trim`,
  hh: `${STORAGE_PREFIX}hh`,
} as const;

export function readJson<T>(key: string): T | null {
  try {
    const raw = localStorage.getItem(key);
    return raw ? (JSON.parse(raw) as T) : null;
  } catch {
    return null;
  }
}

export function writeJson(key: string, value: unknown): void {
  try {
    localStorage.setItem(key, JSON.stringify(value));
  } catch {
    /* quota or private mode: silently ignore */
  }
}
