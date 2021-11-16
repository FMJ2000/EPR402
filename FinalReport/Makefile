##
##  Department of Electrical, Electronic and Computer Engineering.
##  EPR400/2 Final Report - Makefile.
##  Copyright (C) 2011-2021 University of Pretoria.
##

all: finalreport.pdf

finalreport.pdf: *.tex *.bib epr400.cls
	pdflatex finalreport
	bibtex   finalreport
	bibtex   finalreport
	pdflatex finalreport
	pdflatex finalreport

clean:
	rm -f *.aux *.log *.bbl *.blg *.toc
	rm -f *converted-to.pdf summary.pdf finalreport.pdf

## End of File.
