dissertation:
	pdflatex dissertation.tex
	bibtex dissertation
	makeindex dissertation.idx
	pdflatex dissertation.tex
	pdflatex dissertation.tex

.PHONY:clean
.SILENT:clean
clean:
	@-rm -f *.toc *.acn *.aux *.glo *.idx *.lof *.log *.lol *.lot *.pdf *.ist *.bbl *.blg *.ilg
