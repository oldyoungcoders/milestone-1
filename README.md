# milestone-1
files for milestone 1

cd docs 
pandoc milestone_1_report.md -o milestone_1_report.pdf \
  --pdf-engine=xelatex \
  -V mainfont="DejaVu Serif" \
  -V monofont="DejaVu Sans Mono" \
  -V geometry:margin=2.2cm \
  -V colorlinks=true