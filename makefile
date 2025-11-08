# Makefile for compiling a single LaTeX file (main.tex)

TEX_FILE = main
PDF_FILE = $(TEX_FILE).pdf
SRC_FILE = $(TEX_FILE).tex
LATEX_COMPILER = lualatex

.PHONY: all clean

# Default target: builds the PDF
all: $(PDF_FILE)

# Rule to create the PDF from the .tex file
$(PDF_FILE): $(SRC_FILE)
	@echo "--- Compiling $(SRC_FILE) ---"
	$(LATEX_COMPILER) $(SRC_FILE)
	# Run a second time for cross-references/table of contents to stabilize
	$(LATEX_COMPILER) $(SRC_FILE)

# Clean target: removes generated files
clean:
	@echo "--- Cleaning up generated files ---"
	@rm -f *.aux *.log *.out *.toc *.nav *.snm *.fls *.synctex.gz $(PDF_FILE)