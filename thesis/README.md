# Master_thesis_template_latex

# Table of Content (ToC):
To get a correct table of content, run (in Texmaker) "PdfLaTex + View PDF" twice.

# Nomenclature:

## Compilation

### Texmaker
To get a nomenclature, you need to run a command:
- "makeindex %.nlo -s nomencl.ist -o %.nls -t %.nlg"

This command can be added (in Texmaker) as an user defined command. Rebuild "PdfLaTex + View PDF" to add the nomenclature in the ToC.

### VScode:
You need to add a new recipe. The **settings.json** should look similar to this:

```
"latex-workshop.latex.recipes": [
    {
        "name": "pdflatex -> bibtex -> makeindex -> pdflatex * 2",
        "tools": [
        "pdflatex",
        "bibtex",
        "makeindex",
        "pdflatex",
        "pdflatex"
        ]
    }
],

"latex-workshop.latex.tools": [
    {
        "name": "bibtex",
        "command": "bibtex",
        "args": [
            "%DOCFILE%"
        ],
        "env": {}
    },
    {
        "name": "pdflatex",
        "command": "pdflatex",
        "args": [
            "-synctex=1",
            "-interaction=nonstopmode",
            "-file-line-error",
            "%DOC%"
        ],
        "env": {}
    },
    { 
        "name": "makeindex",
        "command": "makeindex",
        "args": [
            "%DOCFILE%.nlo",
            "-s",
            "nomencl.ist",
            "-o",
            "%DOCFILE%.nls"
        ]
    }
],
```

- **Beware**: (maybe a stupid note but I made this mistake) always edit the source file "nomenclature.tex" not the temporary file "thesis.nls"

## How to add a category in Nomenclature:
- Open SPhdThesis.cls 
- Find: 'NOMENCLATURE'
- Then, it should be quite intuitive, just beware of a correct placement of the curly brackets.


# Biblio
**Important note**: At least one citation needs to be in the text. Otherwise, an error occurs.

There are two options:
- \doloiSimpleBib{'file_name'}
- \doloiAutoBib{'file_name'}

First option is simpler, you directly write what will be in the biblio (faster compilation) but more laborious. Second option is for automatic bibliography. You need to build the .tex using a command (in Texmaker) 'PdfLaTeX + Bib(la)tex + PdfLaTeX (x2) + View Pdf' when you want to re-build your biblio. 

# Changing between the print and the screen mode:
Open file "SPhdThesis.cls" and search in the file for: "screen,print" (around line 19). Value in square brackets (for instance "{screen,print}[screen]"), defines the current mode.

# Warnings
## "\@starttoc has already been redefined; tocloft bailingout."
Packages "tocloft" and "parskip" are in a conflict. To get rid of a warning comment:
- \RequirePackage{parskip}
in the "SPhdThesis.cls". 
This will, however, change the look of paragraph indentation. The warning itself should not be an issue.

# Colors
All colors are defined by 'CTU in Prague' house of colors. To see their definitions, find 'SECTION: COLORS' in 'SPhdThesis.cls'.

# How to change font color of the headlines?
I used mainly the color 'ctu-dark-blue' as a headline font color. So in 'SPhdThesis.cls', find all occurances of 'ctu-dark-blue' and replace it by any color you want, for instance, 'ctu-black'. This will, however, also change the color of the hyperlinks.

# Some handy links:
- http://tex.stackexchange.com/questions/13357/fncychap-package-reduce-vertical-gap-space-between-header-and-chapter-heading
- https://tex.stackexchange.com/questions/268406/how-to-color-section-number-and-section-name-with-different-colors/268407#268407
- https://tex.stackexchange.com/questions/8351/what-do-makeatletter-and-makeatother-do
