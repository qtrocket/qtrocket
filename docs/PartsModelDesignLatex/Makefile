# =============================================================================
#  Makefile -- PartsModel design whitepaper
#
#  Pipeline:
#    diagrams/*.puml  --(plantuml -tsvg)-->  *.svg  --(rsvg-convert)-->  out/*.pdf
#    *.tex + out/*.pdf  --(latexmk -pdf -shell-escape)-->  PartPlacement.pdf
#
#  Targets:
#    make            build diagrams then the PDF (default)
#    make diagrams   render only the PlantUML class diagrams to PDF
#    make tools      fetch a self-contained plantuml.jar if missing
#    make clean      remove LaTeX aux + minted cache + diagram SVGs
#    make veryclean  also remove the final PDF and rendered diagram PDFs
#
#  Requirements (all verified present on this system):
#    java, tools/plantuml.jar (auto-fetched), rsvg-convert (or inkscape),
#    latexmk + pdflatex, minted's pygmentize.
# =============================================================================

MAIN         := PartsModel
DIAGDIR      := diagrams
OUTDIR       := $(DIAGDIR)/out
TOOLS        := tools
PLANTUML_JAR := $(TOOLS)/plantuml.jar
PLANTUML_VER := 1.2026.6
PLANTUML_URL := https://github.com/plantuml/plantuml/releases/download/v$(PLANTUML_VER)/plantuml-$(PLANTUML_VER).jar

JAVA         := java
PLANTUML     := $(JAVA) -jar $(PLANTUML_JAR)

# SVG -> PDF converter. rsvg-convert is preferred (clean output); override with
# e.g.  make SVG2PDF="inkscape --export-type=pdf --export-filename"  if needed.
SVG2PDF      ?= rsvg-convert -f pdf -o

LATEXMK      := latexmk -pdf -shell-escape -interaction=nonstopmode -halt-on-error -file-line-error

PUML_SRC     := $(wildcard $(DIAGDIR)/*.puml)
PUML_PDF     := $(patsubst $(DIAGDIR)/%.puml,$(OUTDIR)/%.pdf,$(PUML_SRC))
TEX_SRC      := $(MAIN).tex preamble.tex $(wildcard sections/*.tex) \
                $(DIAGDIR)/tikzstyles.tex $(wildcard $(DIAGDIR)/tikz/*.tex)

.PHONY: all diagrams tools clean veryclean
.SECONDARY: $(patsubst $(DIAGDIR)/%.puml,$(OUTDIR)/%.svg,$(PUML_SRC))

all: $(MAIN).pdf

diagrams: $(PUML_PDF)

tools: $(PLANTUML_JAR)

$(PLANTUML_JAR):
	@mkdir -p $(TOOLS)
	@echo ">> fetching plantuml.jar v$(PLANTUML_VER)"
	@curl -fsSL -o $@ $(PLANTUML_URL)

# PlantUML resolves `!include` relative to the CWD, so we cd into $(DIAGDIR)
# before invoking it (and pass the jar + output dir as absolute paths).
$(OUTDIR)/%.svg: $(DIAGDIR)/%.puml $(DIAGDIR)/_style.iuml | $(PLANTUML_JAR)
	@mkdir -p $(OUTDIR)
	@echo ">> plantuml $< -> $@"
	@cd $(DIAGDIR) && $(JAVA) -jar "$(abspath $(PLANTUML_JAR))" -tsvg -o "$(abspath $(OUTDIR))" "$*.puml"

$(OUTDIR)/%.pdf: $(OUTDIR)/%.svg
	@echo ">> svg2pdf $< -> $@"
	@$(SVG2PDF) $@ $<

# The PDF depends on the rendered diagrams so a clean tree builds in one `make`.
$(MAIN).pdf: $(TEX_SRC) $(PUML_PDF)
	@echo ">> latexmk $(MAIN).tex"
	@$(LATEXMK) $(MAIN).tex

clean:
	@$(LATEXMK) -c $(MAIN).tex >/dev/null 2>&1 || true
	@rm -rf _minted-* $(OUTDIR)/*.svg
	@echo ">> cleaned aux + minted cache + diagram SVGs"

veryclean: clean
	@$(LATEXMK) -C $(MAIN).tex >/dev/null 2>&1 || true
	@rm -f $(MAIN).pdf
	@rm -rf $(OUTDIR)
	@echo ">> removed PDF + rendered diagrams"
