# Torque Tyrants #30819 — Team Website

Static site. No build step: plain HTML, CSS, and JavaScript. Everything the site needs lives in this `website/` folder.

```
website/
├── index.html          # the whole site (single page)
├── assets/
│   ├── css/styles.css
│   ├── js/main.js      # nav, gallery, lightbox, contact form
│   ├── js/robot3d.js   # interactive 3D FTC robot (three.js from CDN)
│   ├── img/            # photos, logo, sponsor logos (.webp)
│   ├── favicon.png
│   └── og-image.jpg    # link-preview image
├── robots.txt
└── .nojekyll
```

## Deploy (point the host at this folder)

- **Vercel**: Import the repo, set **Root Directory** to `website`, Framework Preset "Other", leave build command empty.
- **Netlify**: New site from Git, **Base directory** `website`, publish directory `website`, no build command.
- **Cloudflare Pages**: Build command empty, **Build output directory** `website`.
- **GitHub Pages**: Pages can only serve `/` or `/docs` from a branch, so either rename this folder to `docs`, or use a GitHub Action that uploads `website/` (see `actions/upload-pages-artifact` with `path: website`).

## Preview locally

```
cd website
python -m http.server 8000
```
Then open http://localhost:8000. (Opening `index.html` directly also works, except the 3D model needs a server for its module imports in some browsers.)

## Editing content

- Sponsors: search `sponsor-list` in `index.html`. Logos go in `assets/img/`.
- News / schedule: search `id="news"`.
- Stats: the `data-count` numbers in the Outreach section.
- Donate link: `https://hcb.hackclub.com/donations/start/ftc-torque-tyrants-team-30819` (used by every "Become a sponsor" button).
