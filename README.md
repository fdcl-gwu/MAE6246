# MAE 6246 course site

This repository contains the Fall 2026 public course website for MAE 6246,
Linear Systems and Control.

## Repository and published address

- Repository: `https://github.com/fdcl-gwu/MAE6246`
- Course site: `https://fdcl-gwu.github.io/MAE6246/`

The GitHub Pages source is the `public` directory. The workflow in
`.github/workflows/pages.yml` publishes this directory automatically whenever
the `main` branch is updated.

## Editing the homepage locally

The site is deliberately plain HTML, CSS, and a small amount of JavaScript. It
does not require Quarto, Node.js, or a build step.

- Edit weekly text, dates, topics, examples, and material links in
  `public/index.html`.
- Edit colors, spacing, typography, and responsive layout in
  `public/styles.css`.
- Edit the repository owner, repository name, or branch in
  `public/site-config.js`.
- `public/site.js` constructs Colab links and controls whether material buttons
  are released. It normally does not need to be edited.

To preview the site, run the following from the repository root:

```bash
python3 -m http.server 8000 --directory public
```

Then visit `http://localhost:8000` in a browser. Stop the preview with
`Control-C`.

## Adding and releasing weekly material

1. Put a course-note PDF in `public/notes`, such as `public/notes/week01.pdf`.
2. Put a homework PDF in `public/homework`, such as
   `public/homework/homework01.pdf`.
3. Put the editable notebook in `notebooks`, such as
   `notebooks/week01_pendulum.ipynb`.
4. Find the matching Week 1 links in `public/index.html`.
5. Change `data-released="false"` to `data-released="true"` only for the files
   that are ready for students.

The note and homework buttons open files on the course site. The notebook
button opens the file from GitHub in Google Colab.

## Publishing changes

After previewing your changes:

```bash
git add .github .gitignore public notebooks README.md
git commit -m "Update Week 1 course materials"
git push origin main
```

For the first publication, open the repository's **Settings → Pages** and set
the source to **GitHub Actions**. Subsequent pushes to `main` will publish
automatically. The repository's **Actions** tab shows the publication status.

Avoid editing the generated `gh-pages` deployment directly; edit the files in
`public` and push them to `main` instead.

## Public and private material

Homework statements and starter files may be public. Unreleased solutions,
examinations, grades, student submissions, and student information should
remain in Blackboard or another approved private grading system.
