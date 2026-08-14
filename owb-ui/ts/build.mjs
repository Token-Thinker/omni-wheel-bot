// Esbuild driver for the OWB operator UI.
//
// Two modes:
//   node build.mjs            -> production: minified IIFE -> ../assets/script.js
//   node build.mjs --watch    -> dev: unminified + sourcemap -> ../dist/script.js
//
// The production bundle is what `owb-ui/src/lib.rs` embeds via `include_str!`,
// so it is checked into source control. The dev bundle lives under
// `owb-ui/dist/` which is gitignored.

import { build, context } from 'esbuild';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { readFileSync, writeFileSync } from 'node:fs';

const here = dirname(fileURLToPath(import.meta.url));
const watch = process.argv.includes('--watch');

// Produce a self-contained HTML by inlining the built JS bundle and the CSS
// directly into the page. This eliminates the parallel-asset race the
// embedded picoserve hit when serving 3 separate responses for /, /style.css,
// and /script.js (one of them intermittently got dropped on a fresh page
// load, leaving the phone with an unstyled white screen).
function bundleHtml() {
  const assetsDir = resolve(here, '../assets');
  const html = readFileSync(resolve(assetsDir, 'index.html'), 'utf8');
  const css = readFileSync(resolve(assetsDir, 'style.css'), 'utf8');
  const js = readFileSync(resolve(assetsDir, 'script.js'), 'utf8');
  const inlined = html
    .replace(/<link rel="stylesheet" href="\/style\.css" \/>/,
             `<style>\n${css}\n</style>`)
    .replace(/<script src="\/script\.js"><\/script>/,
             `<script>\n${js}\n</script>`);
  writeFileSync(resolve(assetsDir, 'index.bundled.html'), inlined);
  console.log(`[owb-ui] built assets/index.bundled.html (${inlined.length} bytes)`);
}

const sharedOptions = {
  entryPoints: [resolve(here, 'src/main.ts')],
  bundle: true,
  format: 'iife',
  target: 'es2020',
  loader: { '.ts': 'ts' },
  legalComments: 'none',
  logLevel: 'info',
};

if (watch) {
  const ctx = await context({
    ...sharedOptions,
    outfile: resolve(here, '../dist/script.js'),
    minify: false,
    sourcemap: true,
  });
  await ctx.watch();
  console.log('[owb-ui] esbuild watching, output: owb-ui/dist/script.js');
} else {
  await build({
    ...sharedOptions,
    outfile: resolve(here, '../assets/script.js'),
    minify: true,
    sourcemap: false,
  });
  console.log('[owb-ui] built owb-ui/assets/script.js');
  bundleHtml();
}
