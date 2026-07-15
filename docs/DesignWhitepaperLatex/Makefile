# =============================================================================
#  Makefile -- Architecture & Design whitepaper
#
#  Pipeline:
#    *.tex (main + preamble + sections + TikZ diagrams)
#      --(latexmk -pdf -shell-escape)-->  DesignWhitepaper.pdf
#
#  All diagrams are hand-built TikZ (no PlantUML step, unlike the Part
#  Placement whitepaper this style is inherited from).
#
#  Targets:
#    make            build the PDF (default)
#    make clean      remove LaTeX aux + minted cache
#    make veryclean  also remove the final PDF
#
#  Requirements: latexmk + pdflatex, minted's pygmentize.
# =============================================================================

MAIN     := DesignWhitepaper
DIAGDIR  := diagrams

LATEXMK  := latexmk -pdf -shell-escape -interaction=nonstopmode -halt-on-error -file-line-error

TEX_SRC  := $(MAIN).tex preamble.tex $(wildcard sections/*.tex) \
            $(DIAGDIR)/tikzstyles.tex $(wildcard $(DIAGDIR)/tikz/*.tex)

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
