# =============================================================================
#  Makefile -- QtRocket User Guide
#
#  Pipeline:
#    *.tex (main + preamble + sections) + images/*
#      --(latexmk -pdf -shell-escape)-->  UserGuide.pdf
#
#  No diagram stage: figures are GUI/CLI screenshots under images/ plus the
#  TikZ title banner compiled in line (no PlantUML, unlike the Part Placement
#  whitepaper this style is inherited from).
#
#  Targets:
#    make            build the PDF (default)
#    make clean      remove LaTeX aux + minted cache
#    make veryclean  also remove the final PDF
#
#  Requirements: latexmk + pdflatex, minted's pygmentize.
# =============================================================================

MAIN     := UserGuide

LATEXMK  := latexmk -pdf -shell-escape -interaction=nonstopmode -halt-on-error -file-line-error

TEX_SRC  := $(MAIN).tex preamble.tex $(wildcard sections/*.tex) $(wildcard images/*)

.PHONY: all clean veryclean

all: $(MAIN).pdf

$(MAIN).pdf: $(TEX_SRC)
	@echo ">> latexmk $(MAIN).tex"
	@$(LATEXMK) $(MAIN).tex

clean:
	@$(LATEXMK) -c $(MAIN).tex >/dev/null 2>&1 || true
	@rm -rf _minted-* _minted
	@echo ">> cleaned aux + minted cache"

veryclean: clean
	@$(LATEXMK) -C $(MAIN).tex >/dev/null 2>&1 || true
	@rm -f $(MAIN).pdf
	@echo ">> removed PDF"
