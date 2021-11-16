%%
%%  Department of Electrical, Electronic and Computer Engineering.
%%  EPR400/2 General Report Class File.
%%  Copyright (C) 2011-2021 University of Pretoria.
%%

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{epr400}

%% This class is an extension of the standard book class and will pass
%%   all unknown options to the foundation class
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions\relax
\LoadClass[12pt,a4paper,oneside]{article}

\def\pdfshellescape{1}

%% Establish input relative to a directory.
\RequirePackage{import}

%% Use the "geometry" package to maximize page usage whilst retaining enough
%%   margin space for staples and editorial notes.
\RequirePackage[a4paper,includefoot,nohead,
                top=25mm,left=30mm,right=25mm,bottom=25mm
               ]{geometry}

%% Load support for colour.
\RequirePackage[table]{xcolor}

% COLORS (Tango) Mostly by Philip Bunge
% http://pbunge.crimson.ch/
\definecolor{White}{gray}{0.9}
\definecolor{Black}{gray}{0.0}
\definecolor{LightButter}{rgb}{0.98,0.91,0.31}
\definecolor{LightOrange}{rgb}{0.98,0.68,0.24}
\definecolor{LightChocolate}{rgb}{0.91,0.72,0.43}
\definecolor{LightChameleon}{rgb}{0.54,0.88,0.20}
\definecolor{LightSkyBlue}{rgb}{0.45,0.62,0.81}
\definecolor{LightPlum}{rgb}{0.68,0.50,0.66}
\definecolor{LightScarletRed}{rgb}{0.93,0.16,0.16}
\definecolor{Butter}{rgb}{0.93,0.86,0.25}
\definecolor{Orange}{rgb}{0.96,0.47,0.00}
\definecolor{Chocolate}{rgb}{0.75,0.49,0.07}
\definecolor{Chameleon}{rgb}{0.45,0.82,0.09}
\definecolor{SkyBlue}{rgb}{0.20,0.39,0.64}
\definecolor{Plum}{rgb}{0.46,0.31,0.48}
\definecolor{ScarletRed}{rgb}{0.80,0.00,0.00}
\definecolor{DarkButter}{rgb}{0.77,0.62,0.00}
\definecolor{DarkOrange}{rgb}{0.80,0.36,0.00}
\definecolor{DarkChocolate}{rgb}{0.56,0.35,0.01}
\definecolor{DarkChameleon}{rgb}{0.30,0.60,0.02}
\definecolor{DarkSkyBlue}{rgb}{0.12,0.29,0.53}
\definecolor{DarkPlum}{rgb}{0.36,0.21,0.40}
\definecolor{DarkScarletRed}{rgb}{0.64,0.00,0.00}
\definecolor{Aluminium1}{rgb}{0.93,0.93,0.92}
\definecolor{Aluminium2}{rgb}{0.82,0.84,0.81}
\definecolor{Aluminium3}{rgb}{0.73,0.74,0.71}
\definecolor{Aluminium4}{rgb}{0.53,0.54,0.52}
\definecolor{Aluminium5}{rgb}{0.33,0.34,0.32}
\definecolor{Aluminium6}{rgb}{0.18,0.20,0.21}


%% Enable more advanced character protrusion and font expansion, adjustment
%%   of inter-word spacing and additional kerning, etc.
\RequirePackage[activate={true,nocompatibility},tracking=true,
    kerning=true,spacing=true]{microtype}
\microtypecontext{spacing=nonfrench}

%% Load modern Latin font encoding template T1, as well as additional
%%   symbols.
\RequirePackage{lmodern}
\RequirePackage[T1]{fontenc}
\RequirePackage{textcomp}
\RequirePackage{fix-cm}

%% Use the standard PostScript Type 1 version fonts for Roman, Sans Serif
%%   and Typewriter font classes. This allows the PDF files to be smaller
%%   as the custom TeX fonts do not have to be embedded.
\RequirePackage{mathptmx}
\RequirePackage[scaled=0.92]{helvet}
\RequirePackage{courier}
%   The "times" package does the same as the above all in one go, but is
%   generally considered obsolete.
%\RequirePackage{times}

%% Enable fancy PostScript based characters.
\RequirePackage{pifont}

%% Load AMS mathematics support, fonts and symbols.
\RequirePackage[fleqn]{amsmath}
\RequirePackage{amsfonts}
\RequirePackage{amstext}
\RequirePackage{amssymb}
\RequirePackage{mathtools}

%% Set maths equation number to reset per section
\numberwithin{equation}{section}

%% Yet more mathematics font related modules...
\RequirePackage{bm}
\RequirePackage[mathcal]{eucal}
\RequirePackage{upgreek}

%% Load additional symbols.
\RequirePackage{wasysym}
\RequirePackage{latexsym}
%\RequirePackage{MnSymbol}

%% Define input language as British English.
\RequirePackage[british]{babel}

%% Add calculator for dimensions.
\RequirePackage{calc}

%% Load support for advanced control structures in macros.
\RequirePackage{ifthen}

%% Load support for direct PDF generation.
\RequirePackage{ifpdf}

%% Special spaces in macros.
\RequirePackage{xspace}

%% Load support for external PDF file inclusion.
\RequirePackage[final]{pdfpages}

%% Change the "Contents" header to "Table of Contents". The addto approach
%%   much be used because of the use of Babel.
\addto\captionsbritish{%
  \renewcommand{\contentsname}%
  {TABLE OF CONTENTS}%
  }

%% Remove the "Bibliography" so that the section header can be used.
\addto\captionsbritish{%
  \renewcommand{\bibname}%
    {References}%
}

%% Remove the "Bibliography" so that the section header can be used.
\addto\captionsbritish{%
  \renewcommand{\refname}%
    {References}%
}

%% Redefine the bibliography environment so that the title is displayed
%%   using the standard section command, but made uppercase as is required.
\renewenvironment{thebibliography}[1]
     {\section{\refname}%
      \@mkboth{\refname}{\refname}%
      \list{\@biblabel{\@arabic\c@enumiv}}%
           {\settowidth\labelwidth{\@biblabel{#1}}%
            \leftmargin\labelwidth
            \advance\leftmargin\labelsep
            \@openbib@code
            \usecounter{enumiv}%
            \let\p@enumiv\@empty
            \renewcommand\theenumiv{\@arabic\c@enumiv}}%
      \sloppy
      \clubpenalty4000
      \@clubpenalty \clubpenalty
      \widowpenalty4000%
      \sfcode`\.\@m}
     {\def\@noitemerr
       {\@latex@warning{Empty `thebibliography' environment}}%
      \endlist}

%% Enable fancy headers and set to required format.
\RequirePackage{fancyhdr}

% Make the header wide enough to fit the header bar
\addtolength{\headheight}{12pt}
\addtolength{\headsep}{24pt}
\fancyheadoffset[L]{5mm}
\fancyfootoffset[L]{5mm}

\fancypagestyle{plain}{
  \fancyhf{}
  \renewcommand{\headrulewidth}{0pt}
  \renewcommand{\footrulewidth}{0pt}
  \fancyhead[L]{\fontsize{10}{12}\selectfont\eprthecandidatename}
  \fancyhead[R]{\fontsize{10}{12}\selectfont\eprthepartname}
  \fancyfoot[R]{\fontsize{10}{12}\selectfont\thepage}
}
\pagestyle{plain}
\pagenumbering{arabic}

%% Load support for extended floating environments.
\RequirePackage{float}

%% Allow verbatim file importing
\RequirePackage{alltt}
\RequirePackage{moreverb}

%% Add support for pseudo code for algorithms
\RequirePackage{algorithm}
\RequirePackage{algorithmic}

%% Enable fancy long tables.
\RequirePackage{longtable}

%% Parametrized enumerations
\RequirePackage{enumerate}

%% Package to support advanced theorem
\RequirePackage{theorem}

%% Enable multiple column / row support
\RequirePackage{multicol}
\RequirePackage{multirow}

%% Include support for formatting of large numbers.
\RequirePackage{seqsplit}

%% Support for date / time operations
\RequirePackage{datetime}
% Switch to ISO date displayed form 2000/03/08.
\yyyymmdddate{}

%% For the included project proposal add support for a local bibliography.
\RequirePackage{bibunits}

%% Load package to support standard graphics.
\RequirePackage{graphicx}

%% Load package to support advanced vector graphics.
\RequirePackage{tikz}
\usetikzlibrary{shapes,arrows,positioning,matrix,mindmap,
                shadows,fit,calc,intersections}
\pgfdeclarelayer{background}
\pgfdeclarelayer{foreground}
\pgfsetlayers{background,main,foreground}

%% Required for inserting graphs.
\RequirePackage{pgfplots}
\pgfplotsset{compat=newest}

%% Define shorthand font family selection macros
%%                   Computer Concrete font family: \ccrfamily
\newcommand{\ccrfamily} {\fontencoding{T1}\fontfamily{ccr}\selectfont}
%%                   Computer Modern Roman font family: \cmrfamily
\newcommand{\cmrfamily} {\fontencoding{T1}\fontfamily{cmr}\selectfont}
%%                   Computer Modern Sans font family: \cmssfamily
\newcommand{\cmssfamily} {\fontencoding{T1}\fontfamily{cmss}\selectfont}
%%                   Computer Fibonacci font family: \cmfibfamily
\newcommand{\cmfibfamily} {\fontencoding{T1}\fontfamily{cmfib}\selectfont}
%%                   Computer Dunhill font family: \cmdhfamily
\newcommand{\cmdhfamily} {\fontencoding{T1}\fontfamily{cmdh}\selectfont}
%%                   Computer Modern Bright font family: \cmbrfamily
\newcommand{\cmbrfamily} {\fontencoding{T1}\fontfamily{cmbr}\selectfont}

%%                   Latin Modern font family: \lmrfamily
\newcommand{\lmrfamily} {\fontencoding{T1}\fontfamily{lmr}\selectfont}
%%                   Latin Modern Sans font family: \lmssfamily
\newcommand{\lmssfamily} {\fontencoding{T1}\fontfamily{lmss}\selectfont}

%%                   Pandora Roman font family: \panrfamily
\newcommand{\panrfamily} {\fontencoding{T1}\fontfamily{panr}\selectfont}
%%                   Pandora Sans font family: \pssfamily
\newcommand{\pssfamily} {\fontencoding{T1}\fontfamily{pss}\selectfont}

%%                   Postscript font: Times family: \ptmfamily
\newcommand{\ptmfamily} {\fontencoding{T1}\fontfamily{ptm}\selectfont}
%%                   Postscript font: Palatino family: \pplfamily
\newcommand{\pplfamily} {\fontencoding{T1}\fontfamily{ppl}\selectfont}
%%                   Postscript font: New Century family: \pncfamily
\newcommand{\pncfamily} {\fontencoding{T1}\fontfamily{pnc}\selectfont}
%%                   Postscript font: Bookman family: \pbkfamily
\newcommand{\pbkfamily} {\fontencoding{T1}\fontfamily{pbk}\selectfont}
%%                   Postscript font: Helvetica family: \phvfamily
\newcommand{\phvfamily} {\fontencoding{T1}\fontfamily{phv}\selectfont}
%%                   Postscript font: Avant Garde family: \pagfamily
\newcommand{\pagfamily} {\fontencoding{T1}\fontfamily{pag}\selectfont}
%%                   Postscript font: Courier family: \pcrfamily
\newcommand{\pcrfamily} {\fontencoding{T1}\fontfamily{pcr}\selectfont}
%%                   Postscript font: Utopia family: \putfamily
\newcommand{\putfamily} {\fontencoding{T1}\fontfamily{put}\selectfont}
%%                   Postscript font: Charter family: \bchfamily
\newcommand{\bchfamily} {\fontencoding{T1}\fontfamily{bch}\selectfont}

%% Define new font shapes:
%%                   Outline: \olshape
\newcommand{\olshape} {\fontshape{ol}\selectfont}
%%                   Upright italic: \uishape
\newcommand{\uishape} {\fontshape{ui}\selectfont}

%% Define slanted formatting for Latin phrases.
%%                   a posteriori:    \apost
\newcommand{\apost} {\emph{a posteriori}\xspace}
%%                   a prior:    \aprior
\newcommand{\apriori} {\emph{a priori}\xspace}
%%                   c.:    \circa
\newcommand{\circa} {\emph{c.}\xspace}
%%                   cf.:    \compare
\newcommand{\compare} {\emph{cf.}\xspace}
%%                   crica:    \circal
\newcommand{\circal} {\emph{circa}\xspace}
%%                   e.g.:      \eg
\newcommand{\eg} {\emph{e.g.},\xspace}
%%                   etc.:      \etc
\newcommand{\etc} {\emph{etc.}\@\xspace} % \etc
%%                   et al.:    \etal
\newcommand{\etal} {\emph{et al.}\xspace} % \etal
%%                   et seq.:    \etseq
\newcommand{\etseq} {\emph{et seq.}\xspace}
%%                   i.e.:      \ie
\newcommand{\ie} {\emph{i.e.}\xspace}

%% Miscellaneous
\DeclareRobustCommand{\BibTeX}{\textsc{Bib}\TeX\ }

%% Mathematical Units/Symbols
\newcommand{\leftnorm}{\left\|}
\newcommand{\rightnorm}{\right\|}
\newcommand{\Celsius}{{\ensuremath{\;^{\circ}C}}}    %% Degrees Celsius
\newcommand{\degree}{^{\circ}}
\newcommand{\tr}{{\ensuremath{\operatorname{tr}}}}       %% matrix trace operator
\newcommand{\var}{{\ensuremath{\operatorname{var}}}}     %% variance operator
\newcommand{\sd}{{\ensuremath{\operatorname{sd}}}}       %% standard deviation operator
\newcommand{\mode}{\ensuremath{\operatorname{mode}}}
\newcommand{\m}[1]{\ensuremath{\mathbf{#1}}}
\newcommand{\mt}[1]{\ensuremath{\mathbf{#1}^T}}
\newcommand{\median}[1]{\ensuremath{\underset{#1}{\operatorname{median}}}}
\newcommand{\argmin}[1]{\ensuremath{\underset{#1}{\operatorname{argmin}}}}
\newcommand{\argmax}[1]{\ensuremath{\underset{#1}{\operatorname{argmax}}}}
\newcommand{\expect}[1]{\ensuremath{E[#1]}}
\newcommand{\uv}[1]{\ensuremath{\underline{#1}}}
\newcommand{\iv}{\uv{i}}
\newcommand{\jv}{\uv{j}}
\newcommand{\kv}{\uv{k}}

%% TradeMark/Copyright etc. symbols
\newcommand{\Copyright}{\Pisymbol{psy}{227}\xspace} %% Registered Name
\newcommand{\RegMark}{\Pisymbol{psy}{226}\xspace} %% Registered Name
\newcommand{\TradeMark}{\hspace{1pt}\Pisymbol{psy}{228}\xspace}   %% Registered TradeMark

%% Start each section on a new page.
\newcommand{\sectionbreak}{\clearpage}

%% Section heading setup, per the EPR400/2 style guidelines.
%\RequirePackage{sectsty}
%\sectionfont{\fontsize{18pt}{21.6pt}\fontfamily{cmr}\selectfont\sectionrule{0ex}{0pt}{-0.6ex}{0.4pt}}
%\subsectionfont{\fontsize{14pt}{16.8pt}\fontfamily{cmr}\selectfont}
%\subsubsectionfont{\fontsize{12pt}{14.4pt}\fontfamily{cmr}\fontshape{sl}\selectfont}
%\renewcommand\thesection{\arabic{section}.}
%\renewcommand\thesubsection{\arabic{section}.\arabic{subsection}}
\makeatletter
\renewcommand{\@seccntformat}[1]{\csname the#1\endcsname.\quad}
\makeatother
% \renewcommand\thesection{\arabic{section}.}
% \renewcommand\thesubsection{\arabic{section}.\arabic{subsection}}
\RequirePackage[bf,calcwidth,compact]{titlesec}
\titleformat{\section}[hang]
{\fontsize{16pt}{18.6pt}\selectfont\bfseries} {\thesection.} {5pt} {} [\titleline{\titlerule}]
\titlespacing*{\section}{0pt}{*3}{*3}
% \newcommand{\formatsubsectiontitle}[1]{\MakeUppercase{#1}}
\newcommand{\formatsubsectiontitle}[1]{#1}
\titleformat{\subsection}[display]
{\fontsize{14pt}{16.8pt}\selectfont\bfseries} {} {5pt} {\thesubsection\quad\formatsubsectiontitle}
\titlespacing*{\subsection}{0pt}{-1ex}{*0}
\titleformat{\subsubsection}[display]
{\fontsize{12pt}{14.4pt}\selectfont\slshape\bfseries} {} {5pt} {\thesubsubsection\quad}
\titlespacing*{\subsubsection}{0pt}{*0}{*0}

% \newcommand{\secunderline}[1]{
%     \underline{\smash{\fontsize{14pt}{14pt}\selectfont\bfseries{#1}}}}

%% Caption setup
\RequirePackage[hang]{caption}
\DeclareCaptionFormat{capformat}{#1#2\\#3}
\DeclareCaptionStyle{capstyle}{labelfont=bf,justification=raggedright,labelsep=period,textfont=bf}
\captionsetup{style=capstyle,format=capformat}

%% Enable support for extended list environments, specifically ones
%%   with reduced whitespace.
\RequirePackage{paralist}

%% For the included proposal section the margin lines must be adapted.
\RequirePackage[topline=false, bottomline=false, leftmargin=-5mm,
    rightmargin=-5mm]{mdframed}

% Disable paragraph indent and set default paragraph skip.
\setlength{\parindent}{0 pt}
\setlength{\parskip}{1.5ex} %

%% Set table of contents title.
\RequirePackage{titletoc}
% \makeatletter
% \renewcommand{\tableofcontents}{%
%   \vspace*{-4mm}% reduce space before
%   \setlength{\fboxsep}{10pt}%
%   \framebox[\textwidth]{\fontsize{16pt}{18.6pt}\selectfont\bfseries\contentsname\hfill}
%   \hrule
%   \vspace{2mm}% space below rule
%   \@starttoc{toc}}
% \makeatother

% Space below captions
\setlength{\belowcaptionskip}{10 pt}

% Reset maths style parameters.
\setlength{\abovedisplayskip}{4.5pt plus 1pt minus 1pt}
\setlength{\belowdisplayskip}{4.5pt plus 1pt minus 1pt}
\setlength{\abovedisplayshortskip}{0pt plus 1pt}
\setlength{\belowdisplayshortskip}{0pt minus 1pt}

%% Define automatic macro that gets applied at all font changes to reset
%%   the spacing before and after equations.
\RequirePackage{etoolbox}
\newcommand{\zerodisplayskips}{%
  \setlength{\abovedisplayskip}{4.5pt plus 1pt minus 1pt}
  \setlength{\belowdisplayskip}{4.5pt plus 1pt minus 1pt}
  \setlength{\abovedisplayshortskip}{0pt plus 1pt}
  \setlength{\belowdisplayshortskip}{0pt minus 1pt}
}
\appto{\normalsize}{\zerodisplayskips}
\appto{\small}{\zerodisplayskips}
\appto{\footnotesize}{\zerodisplayskips}

%% Include space between rows in equarray
\setlength{\jot}{5 pt}

%% Allow extra whitespace in lines, decrease number of hyphenations
\sloppy

%% Appendix
\RequirePackage{appendix}
\renewcommand\appendixname{APPENDIX}
\appendixtitletocon
\appendixtocoff
\appendixpageoff
\appendixtitleon

%% UP specific macros
\newcommand{\up}{University of Pretoria}
\newcommand{\ebit}{Faculty of Engineering, Built Environment and Information Technology}
\newcommand{\eeceup}{University of Pretoria. Department of Electrical, Electronic and Computer Engineering.}
\newcommand{\eece}{Department of Electrical, Electronic and Computer Engineering}

% Pull in support for EPS -> PDF conversion for the UP logo.
\ifpdf{}
  \usepackage{epstopdf}
  \pdfadjustspacing=1
\else
\fi

%% Define a new sectioning macro required all main parts of the report.
\newcommand{\eprthepartname}{}
\newcommand{\eprsec}[1]{%
  \renewcommand{\eprthepartname}{#1}
  \setlength{\fboxsep}{10pt}%
  \framebox[\textwidth]{\fontsize{16pt}{18.6pt}\selectfont\bfseries\Large #1\hfill}
  \addcontentsline{toc}{part}{#1}
% {\bfseries\Large #1}\\[-0.75ex]
% \rule{\textwidth}{0.4pt}
}

\newcommand{\eprtitle}[1]{\newcommand{\eprthetitle}{#1}}
\newcommand{\eprcode}[1]{\newcommand{\eprthemodulecode}{#1}}
\newcommand{\eprspectype}[1]{\newcommand{\eprthespectype}{#1}}
\newcommand{\eprcandidatename}[1]{\newcommand{\eprthecandidatename}{#1}}
\newcommand{\eprstudentnumber}[1]{\newcommand{\eprthestudentnumber}{#1}}
\newcommand{\eprdate}[1]{\newcommand{\eprthedate}{#1}}
\newcommand{\eprsupervisor}[1]{\newcommand{\eprthesupervisor}{#1}}
\newcommand{\eprcopynum}[1]{\newcommand{\eprthecopynum}{#1}}

\newcommand{\maketitlepage}{
\thispagestyle{empty}
{
  \renewcommand{\baselinestretch}{1.5}
  \setlength{\parindent}{0mm}
  \setlength{\parskip}{0mm}
  \centering

  \vspace*{\stretch{1}}

  {\bfseries\LARGE\eprthetitle}

  \vspace*{\stretch{1}}

  {\Large Final Report}

  \vspace*{\stretch{1}}

  {\bfseries\Large\eprthecandidatename} \\[1ex]
  {\Large\eprthestudentnumber}

  \vspace*{\stretch{2}}

  {\large
  Submitted as partial fulfilment of the requirements of Project \eprthemodulecode \\
  in the Department of Electrical, Electronic and Computer Engineering \\
  University of Pretoria \\
  }

  \vspace*{\stretch{1}}

  {\large\eprthedate}

  \vspace*{\stretch{1}}

  {\Large Study leader: \eprthesupervisor}

  \vspace*{\stretch{4}}

  % \begin{flushright}
  % {\normalsize \eprthecopynum}
  % \end{flushright}
}
\newpage
}

% Less detailed TOC
\setcounter{tocdepth}{2}

%% Set to single line spacing as required.
\renewcommand{\baselinestretch}{1.0}

\endinput

%% End of File.
