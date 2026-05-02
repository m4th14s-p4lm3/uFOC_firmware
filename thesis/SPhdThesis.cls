% SPhdThesis v1.0
% By Loi Do (doloi@fel.cvut.cz) based on a template by Saurabh Garg (saurabhgarg@mysoc.net)
% Version 1.0 released 2020

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{SPhdThesis}[2013/06/11 v1.0 SPhdThesis class]
\RequirePackage{xkeyval}

% ------------------------------------------------------------------------------------- %
% ----------------------------- MAIN USER-DEFINED OPTIONS ----------------------------- %

% -----------------------------------------------------------------------------
% Define option for formatting for [screen] and print.
% When formatting for screen colors are used for links, tables, and algorithms.
% When formatting for print black color is used for links, tables, and algorithms.
\define@choicekey*[Sg]{PhDThesis}{media}{screen,print}[screen]{\def \SgIntMedia{#1}}
\setkeys[Sg]{PhDThesis}{media}

% -----------------------------------------------------------------------------
% Define option for formatting title page in lower or [upper] case.
\define@choicekey*[Sg]{PhDThesis}{titlecase}{upper,lower}[upper]{\def \SgIntTitleCase{#1}}
\setkeys[Sg]{PhDThesis}{titlecase}

% -----------------------------------------------------------------------------
% Define line spacing. The valid values are [onehalf] and double.
% Note that before using \SgLineSpacing setspace package must be loaded.
\define@choicekey*[Sg]{PhDThesis}{linespacing}[\val\nr]{onehalf,double}[onehalf]{%
	\ifcase\nr\relax
		\def \SgIntLineSpacing{\onehalfspacing}
	\or
		\def \SgIntLineSpacing{\doublespacing}
	\fi
}
\setkeys[Sg]{PhDThesis}{linespacing}

% -----------------------------------------------------------------------------
% Define font size. The valid values are 11pt and 12pt.
\define@choicekey*[Sg]{PhDThesis}{fontsize}{11pt,12pt}[11pt]{\def \SgIntFontSize{#1}}
\setkeys[Sg]{PhDThesis}{fontsize}

% -----------------------------------------------------------------------------
% Define font family to be used. Default is cm.
\define@choicekey*[Sg]{PhDThesis}{font}{cm,times,helvet,palatino}[cm]{\def \SgIntFont{#1}}
\setkeys[Sg]{PhDThesis}{font}

% -----------------------------------------------------------------------------
% Define open. The valid values are 'right' and 'any'.
% Right makes chapters and entries in frontmatter begin only on right hand pages
% or on the next page available. 'any' puts them on the next available page.
\define@choicekey*[Sg]{PhDThesis}{open}[\val\nr]{right,any}[any]{%
	\ifcase\nr\relax
		\def \SgIntOpen {openright}
	\or
		\def \SgIntOpen {openany}
	\fi
}
\setkeys[Sg]{PhDThesis}{open}

% -----------------------------------------------------------------------------
% Define pageside. The valid values are 'oneside' and 'twoside'.
% Specifies whether double or single sided output should be generated.
\define@choicekey*[Sg]{PhDThesis}{pageside}{oneside,twoside}[twoside]{\def \SgIntPageSide{#1}}
\setkeys[Sg]{PhDThesis}{pageside}

% -----------------------------------------------------------------------------
% Define centering of the text on the page. The valid values are 'symmetric' or 'nonsymmetric'
% Specifies whether the text on the page is centered or shifted on one side based on the page number
% NOTE: find occurances of "\SgIntPageCentering" to change the margins
\define@choicekey*[Sg]{PhDThesis}{centering}{symmetric,nonsymmetric}[symmetric]{\def \SgIntPageCentering{#1}}
\setkeys[Sg]{PhDThesis}{centering}

% ---------------------------- END OF USER-DEFINED OPTIONS ---------------------------- %
% ------------------------------------------------------------------------------------- %

% Process all options defined above with default values.
\ProcessOptionsX[Sg]<PhDThesis>

% Finally load the report class.
\LoadClass[a4paper, \SgIntFontSize, \SgIntOpen, \SgIntPageSide]{report}

% ------------------------------------------------------------------------------------- %
% -------------------------------------- PACKAGES ------------------------------------- %
% Include important packages.

\RequirePackage{graphicx, subcaption}                     % Figures.
\RequirePackage{amsmath, amssymb, amsthm}             % Math symbols and fonts.
\RequirePackage{float}                                % Must be done before hyperref.
\RequirePackage[usenames,dvipsnames,hyperref]{xcolor} % For defining colors.
\RequirePackage{ifthen}                               % For comparison.
\RequirePackage{pdfpages}
\RequirePackage{siunitx}
\RequirePackage{mathtools}
\usepackage[utf8]{inputenc}

% ------------------------------------------------------------------------------------- %
% -------------------------------------- APPENDIX ------------------------------------- %
\usepackage[toc,page]{appendix}
\let\appendixpagenameorig\appendixpagename
\renewcommand{\appendixpagename}{\color{ctu-dark-blue}\appendixpagenameorig}

% ------------------------------------------------------------------------------------- %
% --------------------------------- THESIS ASSIGNMENT --------------------------------- %
\newcommand{\ctuAddAssignment}[1]{%
	\newpage
	\thispagestyle{plain}
	\includepdf[pages=-]{#1}
	\SgIntClearDoublePage
}

% ------------------------------------------------------------------------------------- %
% -------------------------------------- COLORS --------------------------------------- %
%% Define the CTU in Prague house style colors.
\definecolor{ctu-dark-blue}{cmyk}{1,0.43,0,0}			% Pantone 300
\definecolor{ctu-black}{cmyk}{0,0,0,1}					% Process Black 100 perc
\definecolor{ctu-white}{cmyk}{0,0,0,0}					% Light green
\definecolor{ctu-light-blue}{cmyk}{0.59,0.17,0,0}		% Pantone 284
\definecolor{ctu-grey}{cmyk}{0,0,0,0.5}					% Process Black 50 perc
\definecolor{ctu-pantone383}{cmyk}{0.35,0,1,0.2}
\definecolor{ctu-red}{cmyk}{0.02,1,0.82,0.06}			% Pantone 186
\definecolor{ctu-orange}{cmyk}{0,0.78,1,0}				% Pantone 166
\definecolor{ctu-pantone326}{cmyk}{0.84,0,0.38,0}		% Cyan?
\definecolor{ctu-pantone5473}{cmyk}{0.86,0.2,0.32,0.53}
\definecolor{ctu-dark-red}{cmyk}{0.13,1,0.5,0.3}		% Pantone 7420
\definecolor{ctu-pink}{cmyk}{0,0.78,0.08,0}				% Pantone 212
\definecolor{ctu-yellow}{cmyk}{0,0.35,1,0}				% Pantone 130

% ------------------------------------------------------------------------------------- %
% --------------------------------------- FONTS --------------------------------------- %
\RequirePackage[T1]{fontenc}  % Use T1 encoded cm-super fonts.
\RequirePackage{microtype}    % Improve typesetting.
\RequirePackage{fix-cm}       % Support for arbitrary font size for cm.
\RequirePackage{lettrine} 	  % Starting a chapter with a big letter

% Specify the format for the section titles in toc.
\newcommand{\SgIntTocSectionFormat}{\bfseries}

% Specify the format for chapter name and number in chapter headings.
\newcommand{\SgIntChapNameFormat}{\fontsize{20}{50}\fontshape{n}\selectfont}
\newcommand{\SgIntChapNumberFormat}{\fontsize{76}{80}\selectfont}

% Specify the format for displaying chapter in fancy header.
\newcommand{\SgIntHeaderFormat}{\large\bfseries}
\newcommand{\SgIntPageNumFormat}{\bfseries}

% Make initial letter
\newcommand{\initial}[2] {
	\lettrine[lines=2]{\color{ctu-dark-blue}#1}{#2}
}

% -----------------------------------------------------------------------------
% CAPTIONS
% Nicer captions for figures and tables. 
% caption package must be included before subfig and hyperref.
% Change the font of the caption to sans serif and make label bold.
\RequirePackage[labelfont=bf]{caption}

% -----------------------------------------------------------------------
% PAGE LAYOUT.

% Use geometry package to set up margins.
% A4 paper is 8.27 x 11.69 inch.
\ifthenelse{\equal{\SgIntPageCentering}{symmetric}}
	{
		\RequirePackage[a4paper, margin=1.0in, top=1in, bottom=1in, includehead, \SgIntPageSide]{geometry}
	}{
		\RequirePackage[a4paper, left=1.25in, right=1in, top=1in, bottom=1in, includehead, \SgIntPageSide]{geometry}
	}


\ifthenelse{\equal{\SgIntOpen}{openright}}
{
	% For adding extra blank page, if necessary, after chapter.
	\RequirePackage{emptypage}
	
	% Define a command to leave a blank page. This is used in frontmatter to add 
	% an empty page between ack, abstract, toc, lof and lot.
	\newcommand{\SgIntClearDoublePage}{\clearpage{\pagestyle{empty}\cleardoublepage}}
}
{
	\newcommand{\SgIntClearDoublePage}{\clearpage}
}


% For disabling paragraph indenting and using a blank line between paragraphs.
\RequirePackage{parskip}

% Set line spacing.
\RequirePackage{setspace}
\SgIntLineSpacing

% Fix footnote spacing
\setlength{\footnotesep}{0.5cm}   % Distance between two footnotes.
\setlength{\skip\footins}{0.5cm}  % Distance between last line of text and first footnote.

% By default Latex centers images vertically on a float page.
% Modify Latex internal variables so that figures are placed from top.
\makeatletter
    \setlength{\@fptop}{0.25cm}
    \setlength{\@fpsep}{1.00cm}
\makeatother

\setlength{\textfloatsep}{1.5cm} % Set the distance between a float and text.
\setlength{\floatsep}{1.0cm}     % Set the distance between two floats.

% Prevent latex from adding extra space between paragraphs so that
% the last line is at the bottom margin on each page.
\raggedbottom

% Define horizontal spacing between subfigures.
\newcommand{\SgIntHSpaceBetweenSubfloats}{\hspace{0.5cm}}



% -----------------------------------------------------------------------
% T A B L E
% 
% Set table layout and design.

\RequirePackage{booktabs, colortbl} % Tables.
\RequirePackage{tabularx}           % Auto column sizing.

\renewcommand{\arraystretch}{1}     % Set space between rows in a table.
\renewcommand{\tabcolsep}{0.20cm}     % Set space between columns in a table.
\heavyrulewidth = 0.15em              % Set width of heavy rules.
\lightrulewidth = 0.07em              % Set width of light rules.
\abovetopsep    = 0.1cm               % Set separation between caption and top rule.
\aboverulesep   = 0.4ex               % Set separation to use above a rule.
\belowrulesep   = 0.4ex               % Set separation to use below a rule.

% Set color for table rules.
%\ifthenelse{\equal{\SgIntMedia}{screen}}{\arrayrulecolor{ctu-dark-blue}}{}
%\ifthenelse{\equal{\SgIntMedia}{print}}{\arrayrulecolor{ctu-dark-blue}}{}



% -----------------------------------------------------------------------
% A L G O R I T H M

\RequirePackage[algo2e, ruled, linesnumbered, algochapter]{algorithm2e}
\DontPrintSemicolon                      % Dont print semicolons at end of lines.
\algoheightrule       = \heavyrulewidth  % Set the width of the top and bottom rules.
\algotitleheightrule  = \lightrulewidth  % Set the width of the middle rule.
\SetAlgoInsideSkip{medskip}              % Set distance between middle rule and algorithm.
\interspacetitleruled = 0.2cm            % Set distance between caption and rules.
\setlength{\algomargin}{2.25em}          % Set the margin of the algorithm text.
\SetNlSkip{1.25em}                       % Set the spacing between line numbers and text.

\newenvironment{SgAlgorithm}[1][t]
{%
	\begin{algorithm2e}[#1]
    \linespread{1.3} % Set the line spacing to one and half.
    \selectfont      % The linespread is effective only after selectfont.%
}
{%
	\end{algorithm2e}
}



% -----------------------------------------------------------------------
% B I B L I O G R A P H Y

% Rename bibliography to references.
\renewcommand{\bibname}{References}

% Define a new command to include the bibliography file and 
% set the formatting options.
\newcommand{\doloiSimpleBib}[1]
{%
	\cleardoublepage                                 % Fix the page number in TOC.
	\phantomsection                            % Fix the link in PDF.
	\addcontentsline{toc}{chapter}{References} % Add the bibliography to TOC.
	%\bibliographystyle{plain}                  % Set the bibliography style.
	%\bibliography{#1}                          % Include the bibliography file.
	\input{#1}
}

\newcommand{\doloiAutoBib}[1]
{%
	\cleardoublepage                                 % Fix the page number in TOC.
	\phantomsection                            % Fix the link in PDF.
	\addcontentsline{toc}{chapter}{References} % Add the bibliography to TOC.
	\bibliographystyle{plain}                  % Set the bibliography style.
	\bibliography{#1}                          % Include the bibliography file.
}

% -----------------------------------------------------------------------
% F A N C Y   H E A D E R
\RequirePackage{fancyhdr}

% Set page style to fancy.
\pagestyle{fancy}

% By default fancyhdr converts the chapter headings to uppercase,
% so restore the chapter casing.
\renewcommand{\chaptermark}[1]{\markboth{\chaptername\ \thechapter.\ #1}{}}

% Define header consisting of Chapter #. Chapter_Name on left side and 
% page number on right side.
% \nouppercase is used on left header to force bibliography in lowercase.
\fancyhead{} 
%\fancyhead[L]{\nouppercase{\SgIntHeaderFormat\leftmark}}
%\fancyhead[R]{\SgIntPageNumFormat\thepage}

% Page numbers on the top left and top right.
\fancyhead[LE,RO]{\SgIntHeaderFormat\thepage}
% Chapter name on the left (even) page.
\fancyhead[RE]{\SgIntHeaderFormat\nouppercase{\leftmark}}
% Section name on the right (odd) page.
\fancyhead[LO]{\SgIntHeaderFormat\nouppercase{\rightmark}}

\renewcommand{\headrulewidth}{1pt}
\addtolength{\headheight}{10pt}

% Disable footer.
\fancyfoot{}
\renewcommand{\footrulewidth}{0pt}

% Define header and footer for plain pages.
\fancypagestyle{plain}
{%
    % Disable header.
    \fancyhead{}
    \renewcommand{\headrulewidth}{0pt}
    
    % Footer contains the page number on right side.
    \fancyfoot{}
    \fancyfoot[R]{\SgIntPageNumFormat\thepage}
    \renewcommand{\footrulewidth}{0pt}
}



% -----------------------------------------
% --------- SECTION: NOMENCLATURE ---------
% -----------------------------------------

% Add nomenclature into TOC
\usepackage[intoc, english]{nomencl}

% -----------------------------------------
% These lines of codes creates the groups

\usepackage{etoolbox}
\renewcommand\nomgroup[1]{%
  \item[\bfseries
  \ifstrequal{#1}{A}{Acronyms}{%
  \ifstrequal{#1}{N}{Lowercase latin letters}{%
  \ifstrequal{#1}{M}{Uppercase latin letters}{%
  \ifstrequal{#1}{G}{Greek letters}{%
  \ifstrequal{#1}{O}{Other Symbols}{}}}}}%
]}
% -----------------------------------------




% -----------------------------------------
% --------- SECTION: TOC, LOF, LOT, LOA ---
% -----------------------------------------
% In progress...
% Change the appearance of toc, lof, lot, etc.

% Using tocloft, the toc can be formatted easily.
\RequirePackage[titles, subfigure]{tocloft}

% Remove dots?
% \renewcommand{\cftdotsep}{\cftnodots}

% Remove dots from list of algorithms.
% This is necessary because we use algorithm2e which mananges its own list of algorithms.
%\makeatletter
%	\renewcommand{\@dotsep}{5000}
%\makeatother

% Format chapter entries differently in toc.
\renewcommand{\cftchapfont}{\SgIntTocSectionFormat}

% Fix the indentation of figure and table entries in the lof, lot, and loa.
\setlength{\cftfigindent}{0in}
\setlength{\cfttabindent}{0in}

\newcommand{\SgAddToc}{\tableofcontents\SgIntClearDoublePage}
%\newcommand{\SgAddLof}{%
%	\newpage
%	\phantomsection % Requires hyperref; this is to fix the link.
%	\addcontentsline{toc}{section}{\numberline{}\hspace{-.35in}{\SgIntTocSectionFormat{}List of Figures}}
%	\listoffigures
%	\SgIntClearDoublePage
%}
\newcommand{\SgAddLot}{%
	\newpage
	\phantomsection % Requires hyperref; this is to fix the link.
	\addcontentsline{toc}{section}{\numberline{}\hspace{-.35in}{\SgIntTocSectionFormat{}List of Tables}}
	\listoftables
	\SgIntClearDoublePage
}
\newcommand{\SgAddLoa}{%
	\newpage
	\phantomsection % Requires hyperref; this is to fix the link.
	\addcontentsline{toc}{section}{\numberline{}\hspace{-.35in}{\SgIntTocSectionFormat{}List of Algorithms}}
	\listofalgorithmes % Note an extra e, it is required because we use algorithm2e.
	\SgIntClearDoublePage
}

  \expandafter\def\csname authoredby#1\endcsname{% Create author colour settings
    \renewcommand{\cftparfont}{\color{\ctu-dark-blue}}}% Subsection colour



% --------------------------------------------------------------
% ------------------ SECTION: CHAPTER HEADING ------------------
% --------------------------------------------------------------
% Change the appearance of chapter headers and section titles.
% Set the format of the part, section and subsection titles.

\RequirePackage{titlesec}
\titlespacing{\chapter}{0pt}{-5ex}{10ex}
\newcommand{\hsp}{\hspace{20pt}}

\titleformat{\part}[display]
	{\LARGE\bfseries\color{ctu-dark-blue}\centering}
	{\textcolor{ctu-black}{Part \thepart}}
	{1em}{}

\titleformat{\chapter}[hang]
	{\Huge\bfseries}
	{\thechapter\hsp\textcolor{ctu-black}{|}\hsp}
	{0pt}
	{\Huge\bfseries\textcolor{ctu-dark-blue}}
	
\titleformat{\section}
  {\Large\bfseries\color{ctu-dark-blue}}
  {\textcolor{ctu-black}{\thesection}}
  {1em}{}
  
\titleformat{\subsection}
  {\large\bfseries\color{ctu-dark-blue}}
  {\textcolor{ctu-black}{\thesubsection}}
  {1em}{}

\titleformat{\subsubsection}
  {\bfseries\color{ctu-dark-blue}}
  {\textcolor{ctu-black}{\thesubsection}}
  {1em}{}


% --------------------------------------------------------------
% ------------------ SECTION: HYPERLINK ------------------------
% --------------------------------------------------------------
% Set hyperlink settings.

\RequirePackage[hyphens]{url} % Embedding URL's in document.

% Enable hyperlinks only in the PDF.
\RequirePackage[bookmarks=true, pdfstartview=Fit, linktoc=page, pdfpagemode=UseNone, unicode]{hyperref}

% Define the colors of the hyperlink.
\ifthenelse{\equal{\SgIntMedia}{screen}}
{%
	%\definecolor{webgreen}{rgb}{0,.5,0} % previously used magenta.
	%\definecolor{webblue}{rgb}{0,0,1}   % previously used orange.
	
	\hypersetup
	{%
		colorlinks = true,
		linkcolor  = {ctu-dark-blue},
		citecolor  = {ctu-dark-blue},
		urlcolor   = {ctu-dark-blue}
	}
}{}
\ifthenelse{\equal{\SgIntMedia}{print}}
{%
	\hypersetup{colorlinks=false,pdfborder={0 0 0}}
}{}

% Define the proerties describing PDF.
\newcommand{\SgIntSetupPdfProps}%
{%
	\hypersetup
	{%
		pdfauthor   = \SgIntAuthor, 
		pdftitle    = \SgIntTitle, 
		pdfsubject  = \SgIntSubject, 
		pdfkeywords = \SgIntKeywords
	}
}



% -----------------------------------------------------------------------
% D O C U M E N T   P R O P E R T I E S
% 
% Important variables used for creating PDF properties and the title page.

% Define empty variables for creating title and setting PDF properties.
\newcommand{\SgIntTitle}{}
\newcommand{\SgIntAuthor}{}
\newcommand{\SgIntAuthorDegrees}{}
\newcommand{\SgIntYear}{}
\newcommand{\SgIntSubject}{}
\newcommand{\SgIntKeywords}{}
\newcommand{\SgIntDegree}{}
\newcommand{\SgIntDepartment}{}
\newcommand{\SgIntFaculty}{}
\newcommand{\SgIntUniversity}{}
\newcommand{\SgIntSupervisor}{}
\newcommand{\SgIntDeclarationDate}{}

% Define commands for setting  the above variables.
\newcommand{\SgSetTitle}[1]{\renewcommand{\SgIntTitle}{#1}\SgIntSetupPdfProps}
\newcommand{\SgSetAuthor}[1]{\renewcommand{\SgIntAuthor}{#1}\SgIntSetupPdfProps}
\newcommand{\SgSetAuthorDegrees}[1]{\renewcommand{\SgIntAuthorDegrees}{#1}}
\newcommand{\SgSetYear}[1]{\renewcommand{\SgIntYear}{#1}}
\newcommand{\SgSetSubject}[1]{\renewcommand{\SgIntSubject}{#1}\SgIntSetupPdfProps}
\newcommand{\SgSetKeywords}[1]{\renewcommand{\SgIntKeywords}{#1}\SgIntSetupPdfProps}
\newcommand{\SgSetDegree}[1]{\renewcommand{\SgIntDegree}{#1}}
\newcommand{\SgSetDepartment}[1]{\renewcommand{\SgIntDepartment}{#1}}
\newcommand{\SgSetFaculty}[1]{\renewcommand{\SgIntFaculty}{#1}}
\newcommand{\SgSetUniversity}[1]{\renewcommand{\SgIntUniversity}{#1}}
\newcommand{\SgSetDeclarationDate}[1]{\renewcommand{\SgIntDeclarationDate}{#1}}
\newcommand{\SgSetSupervisor}[1]{\renewcommand{\SgIntSupervisor}{#1}}


% -----------------------------------------------------------------------
% C L E V E R E F
% 
% Must come as late as possible, especially after hyperref.
\RequirePackage[capitalize]{cleveref}

% Disable the automatic abbreviations of equations and figures.
\crefname{equation}{Equation}{Equations}
\crefname{figure}{Figure}{Figures}
\Crefname{equation}{Equation}{Equations}
\Crefname{figure}{Figure}{Figures}

% Change the way links are produced in PDF documents.
\crefformat{chapter}{#2Chapter~#1#3}
\crefformat{section}{#2Section~#1#3}
\crefformat{figure}{#2Figure~#1#3}
\crefformat{equation}{#2Equation~#1#3}
\crefformat{table}{#2Table~#1#3}
\Crefformat{chapter}{#2Chapter~#1#3}
\Crefformat{section}{#2Section~#1#3}
\Crefformat{figure}{#2Figure~#1#3}
\Crefformat{equation}{#2Equation~#1#3}
\Crefformat{table}{#2Table~#1#3}
\creflabelformat{equation}{#2#1#3}



% -------------------------------------------------
% -------------- SECTION: TITLE PAGE --------------
% -------------------------------------------------
\ifthenelse{\equal{\SgIntTitleCase}{upper}}
{
	\newcommand{\SgIntMakeUpperCase}[1]{\MakeUppercase{#1}}
}
{
	\newcommand{\SgIntMakeUpperCase}[1]{#1}
}
\newcommand{\SgAddTitle}[1]{%
	\newgeometry{left=1.25in, right=1in, top=0.8in, bottom=1in}
	\thispagestyle{empty}
	\begin{center}
		\begin{figure*}[!t]
			\begin{subfigure}[]{2.45in}
				{\color{ctu-dark-blue}{\rule{2.45in}{0.5mm}}}
				{\color{ctu-white}{\rule{2.45in}{0.5mm}}}
				{\color{ctu-white}{\rule{2.45in}{0.5mm}}}
			\end{subfigure}%
			~
			\begin{subfigure}[]{1.0in}
				\includegraphics[width=\textwidth]{#1}
			\end{subfigure}%
			~
			\begin{subfigure}[]{2.45in}
				{\color{ctu-dark-blue}{\rule{2.45in}{0.5mm}}}
				{\color{ctu-white}{\rule{2.45in}{0.5mm}}}
				{\color{ctu-white}{\rule{2.45in}{0.5mm}}}
			\end{subfigure}
		\end{figure*}

		\Large{\SgIntMakeUpperCase{\SgIntUniversity}} \\
		\Large{{\SgIntFaculty}} \\
		\Large{{\SgIntDepartment}} \\
		%\vspace{1in} 
		\vfill
		
		\huge{\color{ctu-dark-blue}\bf{{\SgIntTitle}}}
		
		\vspace{0.5in}
		
		\huge{{\SgIntDegree}}
		
		\vspace*{0.5in}
		
		\huge{{\SgIntAuthor}} 
		\vspace{0.5in}
		
		\Large{Supervisor} \\
		\Large{{\SgIntSupervisor}} \\
		
		\vfill
		
		\Large{\SgIntYear}\\
		{\color{ctu-dark-blue}{\rule{6.05in}{0.5mm}}}
	\end{center}
	
	\SgIntClearDoublePage
	\restoregeometry
}



% -----------------------------------------------------------------------
% D E C L A R A T I O N  P A G E
\newcommand{\SgAddDeclaration}[1][nosignimage]{%
	\newpage
	\thispagestyle{plain}
	\begin{center}
	\vspace*{1.5cm}
	{
		\color{ctu-dark-blue}\Large \bfseries Declaration
	}
	\end{center}

	\vspace{0.5cm}

	\begin{quote}
	I hereby declare that this thesis is my original work and it has been written by me in its entirety. I have duly acknowledged all the sources of information which have been used in the thesis. \newline

	This thesis has also not been submitted for any degree in any university previously.
	\end{quote}

	\ifthenelse{\equal{#1}{nosignimage}}
	{
	    \vspace{2.0cm}
	}
	{
		\vspace{0.5cm}
	}

	\begin{center}
		\ifthenelse{\equal{#1}{nosignimage}}
		{}{\includegraphics{#1}\\[-0.5cm]}
		\rule{5cm}{0.2mm} \\
		\SgIntAuthor \\
		\SgIntDeclarationDate
	\end{center}

	\SgIntClearDoublePage
}



% -----------------------------------------------------------------------
% A C K N O W L E D G M E N T S
% 
% The acknowledgments environment puts a large, bold, centered 
% "Acknowledgments" label at the top of the page. The acknowledgments
% themselves appear in a quote environment, i.e. tabbed in at both 
% sides, and on its own page.
\newenvironment{acknowledgments}%
{%
	\thispagestyle{plain}
	\begin{center}
		\vspace*{1.5cm}
		{%
			\color{ctu-dark-blue}\Large \bfseries Acknowledgments
		}
	\end{center}
	\vspace{0.5cm}
	\begin{quote}
}
{%
    \end{quote}
	\SgIntClearDoublePage
}



% -----------------------------------------------------------------------
% ---------------- SECTION: ENGLISH AND CZECH ABSTRACT ------------------
% -----------------------------------------------------------------------
% The abstract environment puts a large, bold, centered "Abstract" label at
% the top of the page. The abstract itself appears in a quote environment,
% i.e. tabbed in at both sides, and on its own page.
\renewenvironment{abstract}[2]
{%
    \thispagestyle{plain}
    %\newgeometry{left=1.25in, right=1in, top=-1in, bottom=0in}
	\begin{center}
		\vspace*{1.5cm}
		{\color{ctu-dark-blue}\Large\bfseries Abstract}
	\end{center}
	\vspace{0.5cm}

	% English version of the abstract begings here
	#1	
	
	\begin{center}
		\vspace*{1.5cm}
		{\color{ctu-dark-blue}\Large\bfseries Abstrakt}
	\end{center}
	\vspace{0.5cm}	
	
	% Czech version of the abstract begings here
	#2
	
	%\begin{quote}
}
{%
    %\end{quote}
	\SgIntClearDoublePage
	
}


% ---------------------------------------------------------------
% -------------------- SECTION: FRONT MATTER --------------------
% ---------------------------------------------------------------
% The frontmatter environment set the page numbering to lowercase roman for 
% ack, abstract, toc, lof, lot, loa, etc. It also resets page numbering for the 
% remainder of thesis (arabic, starting at 1).
\newenvironment{frontmatter}
{%
 	\setcounter{page}{1}
	\renewcommand{\thepage}{\roman{page}}
}
{%
	\clearpage
	\renewcommand{\thepage}{\arabic{page}}
	\setcounter{page}{1}
	\SgIntClearDoublePage
}




% -------------------------------------------------
% -------------------SECTION: COVER ---------------
% -------------------------------------------------
% In progress...

\RequirePackage{tikz}
\usetikzlibrary{calc}

\newcommand*\coverimage[1]{\def\@cover@image{#1}}
\newcommand*\makecover[1][]{%
	
    \setkeys{cover}{#1}%
    \clearpage%
	
	\newgeometry{margin=0pt}
    \thispagestyle{empty}%
	
	% ------------------------------------------------------------
		% ------------------ COVER, FRONT
	\begin{tikzpicture}[remember picture,overlay]	
		\node at (current page.north east)[anchor=north east,inner sep=0pt]{
			%\includegraphics{cover/front_part.png}
		};
	
		\node at (current page.south west)[anchor=south west,inner sep=2cm]{
			\includegraphics[width=0.5\textwidth]{cover/cover.pdf}
		};
		
		%\coordinate (top left) at ($(current page.north))
		%\coordinate (corner) at ($(top left));
		%\coordinate (front top left) at (corner);
		\draw[fill=ctu-dark-blue, ctu-dark-blue] (2cm,-2cm) rectangle (10cm, -5cm);
	\end{tikzpicture}%
	
	% ------------------------------------------------------------
		% ------------------ RESTORE
	\restoregeometry%
}












