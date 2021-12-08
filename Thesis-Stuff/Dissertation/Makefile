dissertation:
	pdflatex -interaction=nonstopmode dissertation.tex
	bibtex dissertation
	makeindex dissertation.idx
	pdflatex -interaction=nonstopmode dissertation.tex
	pdflatex -interaction=nonstopmode dissertation.tex

.PHONY:clean
.SILENT:clean
clean:
	@-rm -f *.toc *.acn *.aux *.glo *.idx *.lof *.log *.lol *.lot *.pdf *.ist *.bbl *.blg *.ilg
