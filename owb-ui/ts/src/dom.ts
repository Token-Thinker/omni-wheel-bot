// Tiny typed querySelector helpers. Centralises null-handling so the rest
// of the codebase can stay narrow without sprinkling `as` casts.

export function byId<T extends HTMLElement = HTMLElement>(id: string): T | null {
  return document.getElementById(id) as T | null;
}

export function requireId<T extends HTMLElement = HTMLElement>(id: string): T {
  const el = document.getElementById(id);
  if (!el) throw new Error(`owb-ui: missing element #${id}`);
  return el as T;
}

export function queryAll<T extends Element = Element>(selector: string): NodeListOf<T> {
  return document.querySelectorAll<T>(selector);
}

export function setText(el: HTMLElement | null, text: string): void {
  if (el) el.textContent = text;
}

export function setAttr(el: HTMLElement | null, name: string, value: string): void {
  if (el) el.setAttribute(name, value);
}
