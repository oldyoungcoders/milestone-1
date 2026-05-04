#!/usr/bin/env bash
# Build docs/literature_review.pdf from docs/literature_review.md
# using pandoc + tectonic (a self-contained modern LaTeX engine).
#
# One-time install (already done):
#     conda install -c conda-forge pandoc tectonic
#
# Run from anywhere:
#     bash scripts/build_pdf.sh
#
# Output:
#     docs/literature_review.pdf  (justified text, journal-style typography,
#                                  proper LaTeX math, embedded PNG figures)
#
# Notes:
#   * Mermaid blocks in the .md are stripped before pandoc sees the file —
#     LaTeX cannot render mermaid.  Matplotlib PNGs (observer_stack_sag.png,
#     option_D_empc_mpc_cascade.png) come through normally.
#   * Tectonic auto-downloads missing LaTeX packages on first run; expect
#     a slow first build (~1-2 min) and fast subsequent builds.
#
#    * bash scripts/build_pdf.sh docs/literature_review.md 2>&1 | tail -3 && ls -la docs/literature_review.pdf   

set -euo pipefail

source ~/anaconda3/etc/profile.d/conda.sh
conda activate dompc_py11

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$SCRIPT_DIR")"
cd "$REPO"

command -v pandoc   >/dev/null || { echo "ERROR: pandoc not on PATH (env: $CONDA_DEFAULT_ENV)" >&2; exit 1; }
command -v tectonic >/dev/null || { echo "ERROR: tectonic not on PATH" >&2; exit 1; }

INPUT="${1:-docs/literature_review.md}"
# Output PDF lives next to the input, with .md -> .pdf
OUTPUT="${INPUT%.md}.pdf"
[[ -f "$INPUT" ]] || { echo "ERROR: input not found: $INPUT" >&2; exit 1; }

# Strip mermaid blocks before pandoc sees the file.  We replace each block
# with a single italic note so the figure number stays sensible in the PDF.
TMP_MD="$(mktemp --suffix=.md)"
trap 'rm -f "$TMP_MD" header.tex' EXIT

# Strip mermaid blocks (kept for safety; current .md has none).
awk '
    BEGIN { skip = 0 }
    /^```mermaid$/   { skip = 1; print "*[Figure rendered as a mermaid diagram on GitHub — see docs/literature_review.md for the live diagram.]*"; next }
    skip && /^```$/  { skip = 0; next }
    !skip            { print }
' "$INPUT" > "$TMP_MD"

# Rewrite relative paper / cross-doc links to absolute GitHub URLs so
# they remain clickable when the PDF is viewed inside the GitHub PDF
# previewer (relative paths don't resolve in that context).  PNG image
# references are deliberately NOT rewritten — pandoc resolves those
# locally to embed the figures in the PDF.
# Replace Unicode box-drawing / arrow characters (used in ASCII art diagrams)
# with plain-ASCII fallbacks.  The lmmono PDF font lacks these glyphs and
# tectonic emits "missing character" warnings.  GitHub markdown still gets
# the prettier Unicode because we operate on the temp file only.
python - "$TMP_MD" <<'PY'
import sys
from pathlib import Path
p = Path(sys.argv[1])
text = p.read_text(encoding="utf-8")
mapping = {
    "─": "-", "━": "-",
    "│": "|", "┃": "|",
    "┌": "+", "┍": "+", "┎": "+", "┏": "+",
    "┐": "+", "┑": "+", "┒": "+", "┓": "+",
    "└": "+", "┕": "+", "┖": "+", "┗": "+",
    "┘": "+", "┙": "+", "┚": "+", "┛": "+",
    "├": "+", "┤": "+", "┬": "+", "┴": "+",
    "┼": "+",
    "╱": "/", "╲": "\\",
    "●": "*", "○": "o",
    "▼": "v", "▲": "^", "►": ">", "◄": "<",
    "≈": r"$\approx$",
}
for src, dst in mapping.items():
    text = text.replace(src, dst)
p.write_text(text, encoding="utf-8")
PY

GH_BASE="https://github.com/oldyoungcoders/milestone-1/blob/main"
# 1. paper PDFs:   ](../papers/foo.pdf)  ->  absolute GitHub URL
sed -i "s|](\.\./papers/|](${GH_BASE}/papers/|g" "$TMP_MD"
# 2. cross-doc .md links (with optional #anchor):
#       ](foo.md)        ->  ](GH_BASE/docs/foo.md)
#       ](foo.md#sec)    ->  ](GH_BASE/docs/foo.md#sec)
sed -E -i "s|]\(([a-zA-Z0-9_-]+\.md(#[^)]*)?)\)|](${GH_BASE}/docs/\1)|g" "$TMP_MD"

# LaTeX header for typography:
#   * microtype + parskip for clean justified paragraphs
#   * babel English for hyphenation
#   * graphicx for the PNG figures
#   * caption for proper figure captions
#   * hyperref for clickable links
cat > header.tex <<'TEX'
% babel + hyperref are loaded automatically by pandoc.
% This header only adds typography + structural niceties.
% \lt and \gt are KaTeX-style aliases used in the markdown for
% GitHub-rendered math; provide them here so the same source compiles in LaTeX.
\providecommand{\lt}{<}
\providecommand{\gt}{>}
\usepackage{microtype}
\usepackage{parskip}
\usepackage{caption}
\captionsetup{font=small,labelfont=bf}
\usepackage{booktabs}
\usepackage{enumitem}
\setlist{topsep=2pt,itemsep=1pt,parsep=0pt}
\usepackage{float}
\renewcommand{\arraystretch}{1.15}
% Shrink wide tables to footnotesize so wide comparison tables fit the page
\usepackage{etoolbox}
\AtBeginEnvironment{longtable}{\footnotesize}
\AtBeginEnvironment{tabular}{\footnotesize}
TEX

pandoc "$TMP_MD" \
    --from=gfm+tex_math_dollars+raw_html \
    --to=pdf \
    --pdf-engine=tectonic \
    --resource-path="docs:." \
    --include-in-header=header.tex \
    -V documentclass=article \
    -V geometry:margin=2.2cm \
    -V fontsize=11pt \
    -V colorlinks=true \
    -V linkcolor=NavyBlue \
    -V urlcolor=NavyBlue \
    -V citecolor=NavyBlue \
    -V title="Literature Review — Improved Control of SAG Mill Grinding Circuit Based on Disturbance Observer-Assisted Model Predictive Control with Two-Layer Set-Point Optimisations" \
    -V lang=en \
    --toc --toc-depth=3 \
    --output="$OUTPUT"

echo "Wrote $OUTPUT"
