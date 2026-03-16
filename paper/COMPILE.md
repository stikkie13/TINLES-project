# Compile the Paper

## Quick Commands
```bash
cd paper
make          # Compile PDF
make view     # Compile and open
make clean    # Remove temp files
```

## Manual Compile
```bash
cd paper
xelatex paper.tex
bibtex paper
xelatex paper.tex
xelatex paper.tex
```