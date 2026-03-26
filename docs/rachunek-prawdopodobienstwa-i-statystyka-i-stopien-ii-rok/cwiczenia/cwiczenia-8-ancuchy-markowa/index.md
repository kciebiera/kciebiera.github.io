---
title: "Ćwiczenia 8: Łańcuchy Markowa"
source_url: "http://smurf.mimuw.edu.pl/node/772"
source_kind: "html"
---

<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@4/tex-mml-chtml.js"></script>

# Ćwiczenia 8: Łańcuchy Markowa

## Zadanie 1

Oblicz średnie czasy powrotu dla wszystkich stanów łańcucha Markowa o macierzy przejścia  

$P = \left(\begin{array}{cc} \frac{1}{4} & \frac{3}{4} \\ \frac{2}{3} & \frac{1}{3}\\ \end{array}\right)$  

 wprost oraz korzystając z twierdzenia ergodycznego.

## Zadanie 2

Dwóch graczy rzuca symetryczną monetą. Jeden obstawia, że najpierw pojawi się ciąg OOR, drugi - że ROO. Jakie prawdopodobieństwo wygranej ma każdy z graczy i jaki jest oczekiwany czas gry?

## Zadanie 3

Na szalce $2\times2$ hodujemy bakterie. Każde pole może zawierać jedną bakterię. Początkowo kolonia składa się z jednej bakterii. Co sekundę (jednocześnie) każda bakteria umiera z prawdopodobieństwem $\frac{1}{2}$, a na każdym z pustych pól sąsiadujących w poprzedniej sekundzie z co najmniej jedną bakterią z prawdopodobieństwem $\frac{1}{2}$ pojawia się nowa bakteria. Oblicz oczekiwany czas życia kolonii i prawdopodobieństwo tego, że kolonia kiedykolwiek zapełni całą szalkę.

## Zadanie 4

Dwaj gracze rzucają na przemian symetryczną monetą. Wygrywa ten, który wyrzuci orła bezpośrednio po orle wyrzuconym przez poprzednika. Jakie jest prawdopodobieństwo tego, że wygra pierwszy gracz? Jaka jest oczekiwana liczba rzutów w całej grze?

## Zadanie 5

Cztery mrówki znajdują się w jednym wierzchołku czworościanu. Co sekundę każda z nich z prawdopodobieństwem $\frac{1}{4}$ przechodzi do sąsiedniego wierzchołka lub pozostaje na miejscu. Jaka jest oczekiwana liczba zajętych przez mrówki wierzchołków po upływie 1 sekundy? Jaki jest oczekiwany czas pierwszego zajęcia wszystkich wierzchołków jednocześnie?

## Zadanie 6

Dwie cząstki znajdują się w przeciwległych wierzchołkach sześcianu. Co sekundę każda z nich z prawdopodobieństwem $\frac{1}{4}$ przechodzi do sąsiedniego wierzchołka lub pozostaje na miejscu. Jaki jest oczekiwany czas kolizji cząstek (tzn. spotkania cząstek w jednym wierzchołku)?

## Zadanie 7

Przy okrągłym stole siedzi trzech graczy $A, B, C$, każdy z jednym żetonem. W każdym kroku gry, jednocześnie każdy gracz z co najmniej jednym żetonem z prawdopodobieństwem $\frac{1}{2}$ przekazuje żeton graczowi po prawej. Grę wygrywa gracz, który zgromadzi wszystkie żetony. Ile wynosi oczekiwany czas gry?

## Zadanie 8 (Problem ruiny gracza)

Gracz rozpoczyna grę z kapitałem $k$ zł. W każdym kroku gry wygrywa 1zł z prawdopodobieństwem $p = \frac{1}{2}$, i z tym samym prawdopodobieństwem $1-p = \frac{1}{2}$ przegrywa 1zł. Gracz kończy grę, gdy albo wygra fortunę (= $n$ zł) albo zbankrutuje. Oblicz:

1. prawdopodobieństwo wygrania fortuny, oraz
2. średnią liczbę kroków do zakończenia gry.

## Zadanie 9 (Model Ehrenfestów)

W dwóch komorach znajdują się cząstki, w jednej $k$, w drugiej $n-k$. W każdym kroku jedna losowo wybrana cząstka zmienia komorę. Oblicz graniczny rozkład liczby cząstek w pierwszej komorze (t.j. rozkład stacjonarny odpowiedniego łańcucha Markowa). Jaki jest średni czas powrotu dla stanu w którym wszystkie cząstki są w pierwszej komorze?
