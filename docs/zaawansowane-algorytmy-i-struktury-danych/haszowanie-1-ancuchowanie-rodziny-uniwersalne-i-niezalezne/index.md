---
layout: 'default'
title: 'Haszowanie 1: łańcuchowanie, rodziny uniwersalne i niezależne'
page_url: 'http://smurf.mimuw.edu.pl/node/1149'
source_url: 'http://smurf.mimuw.edu.pl/node/1149'
source_kind: 'html'
uses_math: true
render_with_liquid: false
rendered_file: 'lectures/zaawansowane-algorytmy-i-struktury-danych/haszowanie-1-ancuchowanie-rodziny-uniwersalne-i-niezalezne/rendered.html'
---

# Haszowanie 1: łańcuchowanie, rodziny uniwersalne i niezależne

Zajmiemy się jednym z podstawowych problemów algorytmicznych: problemem słownika. W tym rozdziale omawiamy jedno z bardziej efektywnych podejść do tego problemu, zwane haszowaniem (warto wspomnieć, że problem słownika jest głównym, ale nie jedynym zastosowaniem haszowania). Przypomnijmy krótko na czym polega problem słownika. 
## Problem słownika

Należy opisać strukturę danych, która reprezentuje pewien zbiór elementów $S$ i udostępnia następujące operacje:

- Lookup($x$) -- czy $x \in S$?
- Insert($x$) -- $S:=S \cup \{x\}$
- Remove($x$) -- $S:=S \setminus \{x\}$

Zakładamy, że elementy zbioru $S$ pochodzą z pewnego *uniwersum* $U$, tzn.  $S\subseteq U$. W zastosowaniach elementy $S$ są zwykle liczbami lub napisami. W dalszych rozważaniach będziemy przyjmować, że $U$ jest zbiorem liczb naturalnych z ograniczonego zakresu, tzn.  $U=\{0,\ldots,\vert{}U\vert{}-1\}$. Nie jest to duże ograniczenie, gdyż w ten sposób możemy reprezentować także np. napisy o ograniczonej długości czy liczby wymierne o ograniczonej precyzji.

Będziemy również zakładać, że znana jest liczba $n$ taka, że $\vert{}S\vert{}\le n$.

## Haszowanie: główny pomysł

W znanych nam drzewiastych rozwiązaniach problemu słownika, chcąc wyszukać element w strukturze danych, algorytm "błądzi" po strukturze danych podążając za kolejnymi wskaźnikami. W haszowaniu chcemy skrócić ten proces: z grubsza rzecz biorąc, dla każdego potencjalnego elementu $x$ chcielibyśmy natychmiast (ewentualnie wykonując proste obliczenia w czasie stałym) poznać miejsce, w którym on się znajduje. W najprostszej wersji, mamy *tablicę  haszującą*

$T[0, \ldots, m-1],$

oraz *funkcję haszującą*

$h: U \longrightarrow \{0,\ldots,m-1\}.$

Ponieważ nie chcemy, aby nasza struktura zajmowała istotnie więcej miejsca niż jest to konieczne, zwykle $m = O(n)$, np. $m = 2n$ lub $m = n$. Potencjalnie $h$ może być dowolną funkcją, choć dla nas użyteczne będą takie, których wartość możemy wyliczyć w czasie stałym. Generalnie, elementu $x\in U$ będziemy szukać w komórce $T[h(x)]$. Oczywiście zwykle uniwersum $U$ jest dużo większe niż $m$, więc niezależnie od tego jakiej funkcji haszującej $h$ użyjemy, będzie się zdarzać, że dwa elementy $x$ i $y$ zostaną odwzorowane w tę samą komórkę tablicy $T$, tzn. $h(x)=h(y)$. Taką sytuację nazywamy *konfliktem*. Zaczynamy od najprostszego, ale całkiem skutecznego sposobu rozwiązywania konfliktów: haszowania przez łańcuchowanie.

## Haszowanie przez łańcuchowanie

W haszowaniu przez łańcuchowanie dla każdego $j$ tablica $T$ w komórce $T[j]$  zawiera *listę* takich elementów $x\in S$, że $h(x) = j$. Teraz możemy bardzo łatwo zaimplementować operacje słownikowe: dla danego $x$ wykonujemy odpowiednią operację na liście $T[h(x)]$ (Lookup -- sprawdzamy czy lista $T[h(x)]$ zawiera element; Insert -- wstawienie elementu na listę $T[h(x)]$; Remove -- usunięcie elementu z listy $T[h(x)]$).

W ten sposób czas wykonania operacji na elemencie $x$ jest proporcjonalny do długości listy $T[h(x)]$. Chcielibyśmy jakoś ograniczyć długość list elementów odwzorowanych w tę samą komórkę. Widać, że wszystko zależy od tego, jak dobrze dobierzemy funkcję $h$. Niektóre funkcje $h$ są w oczywisty sposób ,,złe'', np.  funkcje stałe. Czy istnieją ,,dobre'' funkcje?

Gdybyśmy znali z góry zbiór $S$ być może może udałoby się nawet sprawić, aby każdy element $S$ został odwzorowany w inną komórkę tablicy $T$ (w dalszej części wykładu zobaczymy, że istotnie jest to możliwe). Problem polega na tym, że w naszym problemie słownika to funkcję $h$ ustalamy z góry, przed pojawieniem się ciągu operacji i chcielibyśmy, żeby w każdej chwili w przyszłości elementy $S$ ,,w miarę jednostajnie'' rozproszyły się po $\{0,\ldots,m-1\}$. Naturalnymi kandydatkami mogą być takie funkcje, które jednostajnie rozpraszają $U$, tzn. dla dowolnych $y_1,y_2\in\{0,\ldots,m-1\}$, $\vert{}h^{-1}(y_1)\vert{} \approx  \vert{}h^{-1}(y_2)\vert{}$, np. $h(x) = x \mod m$. Widać, że takie funkcje działałyby dobrze dla losowych danych.

Często dane, które pojawiają się w praktyce są faktycznie zbliżone do losowych. My jednak stawiamy sobie za cel rozwiązanie, które będzie dobre *niezależnie od danych*. Od razu spotyka nas rozczarowanie. Zauważmy bowiem, że

**Uwaga**  

Dla $\vert{}U\vert{}\gg m$, każda funkcja $h: U \mapsto \{0,\ldots,m-1\}$ jest *zła*, tzn. złośliwy przeciwnik, jeśli będzie znał $h$, może tak dobierać elementy dodawane do $S$, że $h(x) = h(y)$ dla każdej pary $x,y \in S$.

W 1977r Carter i Wegman wpadli na prosty pomysłem na poradzenie sobie z tym problemem. Mianowicie, funkcję haszującą będziemy losowali z pewnej rodziny funkcji -- w ten sposób przeciwnik będzie zmuszony się zabezpieczyć przed całym zbiorem funkcji, a to mu się nie uda. Z pewnym małym prawdopodobieństwem algorytm może wprawdzie dalej działać długo, jednakże to prawdopodobieństwo nie zależy od danych. W następnym rozdziale zastanowimy się, jakie warunki powinny spełniać takie rodziny funkcji i podamy kilka przykładów takich rodzin.

## Uniwersalne i niezależne rodziny funkcji haszujących

**Przykład 1**  

Załóżmy, że $h$ jest losowana jednostajnie ze zbioru wszystkich funkcji, czyli z $\{0,\ldots,m-1\}^U$. Odpowiada to wylosowaniu niezależnie dla każdego $x \in U$ wartości $h(x) \in \{0,\ldots,m-1\}$.  

Zauważmy, że wówczas  

$$
\mathbb{P}\left[h(x) = h(y) \right] = \left\{ \begin{array}{ll} 1& \mbox{gdy } x = y \\  \frac{1}{m} & \mbox{ gdy } x \neq y .\end{array} \right.
$$  

Stąd, dla dowolnego $x\in U$:  

$$
\mathbb{E} \vert{}h(x)\vert{} = \mathbb{E}\left[\sum_{y \in S} \mathbf{1}_{h(x) = h(y)} \right]= \sum_{y \in S} \mathbb{P}\left[h(x) = h(y)\right] = \left\{ \begin{array}{ll} \frac{n}{m}& \mbox{gdy } x \notin S \\ 1+ \frac{n-1}{m} & \mbox{gdy } x \in S.\end{array} \right.
$$  

W powyższych rachunkach korzystamy z liniowości wartości oczekiwanej; notacja $\mathbf{1}_{A}$ oznacza indykator zdarzenia $A$, tzn. zmienną losową, która przyjmuje wartość 1 gdy $A$ się wydarzy i $0$ w przeciwnym przypadku.  Dla $m = \Omega(n)$ otrzymane wyrażenie jest $O(1)$, co daje oczekiwany czas stały wszystkich operacji słownikowych. Udało się więc przechytrzyć złośliwego przeciwnika. Ale czy na pewno?

Powstaje pytanie jak szybko wylosować taką funkcję oraz ile pamięci potrzeba, aby ją reprezentować. Pierwsze, co przychodzi do głowy to reprezentowanie $h$ jako tablicy $A[1 \ldots \vert{}U\vert{}]$ liczb od 0 do $m-1$. Jeśli jednak mamy do dyspozycji tyle pamięci (i czasu na inicjalizację) to nie musimy używać haszowania -- słownik możemy zaimplementować jako tablicę bitową rozmiaru $\vert{}U\vert{}$. Z drugiej strony, jeśli naprawdę uprzemy się, żeby $h$ losować ze zbioru *wszystkich*  funkcji, lepsze rozwiązanie nie istnieje: do przechowywania $h$ potrzebujemy co najmniej $\vert{}U\vert{}\log_2 m$ bitów (za pomocą mniej niż $\vert{}U\vert{}\log_2 m$ bitów możemy reprezentować mniej niż $2^{\vert{}U\vert{}\log_2 m}=m^{\vert{}U\vert{}}$ funkcji, czyli nie wszystkie).

Przyjrzyjmy się rachunkom z powyższego przykładu i zastanówmy się czy coś da się jeszcze z niego uratować. Zauważmy, że nie potrzebowaliśmy pełnej losowości, a jedynie własności $\mathbb{P}\left[h(x) = h(y)\right] \leq \frac{1}m$ dla $x \neq y$. Tę cechę dostrzegli w 1977 roku Carter i Wegman. Podali oni następującą definicję.

**Definicja**  

Niech $\mathcal{H} \subseteq \{0,\ldots,m-1\}^{U}$. Powiemy, że $\mathcal{H}$ jest rodziną *$(\alpha,k)$-uniwersalną*, gdy po wybraniu *losowej* funkcji $h \in \mathcal{H}$ (tzn. jednostajnie, każdą z równym prawdopodobieństwem), dla dowolnych parami różnych $x_1, x_2, \ldots, x_k \in U$ zachodzi  

$$
\mathbb{P}[h(x_1) = h(x_2) = \ldots = h(x_k)] \leq \frac{\alpha}{m^{k-1}}.
$$

Rodzinę $(1,k)$-uniwersalną nazywamy krótko *$k$-uniwersalną*, natomiast na rodzinę 2-uniwersalną mówimy jeszcze krócej: *rodzina uniwersalna*. Z naszych wcześniejszych rozważań wynika, że

**Twierdzenie 2**  

*Jeśli używamy haszowania łańcuchowego z funkcją haszującą $h$ wybraną losowo z uniwersalnej rodziny funkcji haszujących, to oczekiwane czasy wszystkich operacji słownikowych są stałe.*

Rozważmy jeszcze jedną naturalną definicję. Mówimy, że rodzina $\mathcal{H}$ jest rodziną *jednostajną*, gdy po wybraniu *losowej* funkcji $h \in \mathcal{H}$, dla dowolnego $x \in U$ zmienna losowa $h(x)$ ma rozkład jednostajny, tzn. dla każdego $a \in  \{0,\ldots,m-1\}$ zachodzi $\mathbb{P}[h(x)=a] = \frac{1}{m}$. Widzimy, że jednostajność jest również cechą pożądaną, jednakże łatwo sobie uświadomić, że nie gwarantuje ona efektywności operacji słownikowych.

**Zadanie**  

 Podaj przykład jednostajnej rodziny funkcji haszujących $\mathcal{H}$ oraz ciągu $n$ operacji Insert oraz Lookup, który wykona się w czasie  $\Omega(n^2)$ niezależnie od tego, jaka funkcja haszująca $h$ zostanie wylosowana z $\mathcal{H}$.

W niektórych zastosowaniach wygodnie jest korzystać z rodzin funkcji posiadających jeszcze silniejsze własności niż rodziny uniwersalne.

**Definicja**  

Niech $\mathcal{H} \subseteq \{0,\ldots,m-1\}^{U}$.  

Powiemy, że $\mathcal{H}$ jest rodziną *$k$-niezależną* (lub *silnie $k$-uniwersalną*), gdy po wybraniu losowej funkcji $h \in \mathcal{H}$, dla dowolnych parami różnych $x_1, x_2, \ldots, x_k \in U$ i dowolnych $y_1, y_2, \ldots , y_k \in \{0,\ldots,m-1\}$ zachodzi  

$$  

\mathbb{P}[h(x_1) = y_1 \wedge  h(x_2) = y_2 \wedge \ldots \wedge h(x_k) = y_k] = \frac{1}{m^{k}}.  

$$  

Równoważnie możemy to wyrazić jako koniunkcję dwóch warunków (ćwiczenie: pokaż równoważność):

- **($k$-niezależność zmiennych)** dla dowolnych parami różnych $x_1, \ldots, x_k \in U$ zmienne losowe $h(x_1), \ldots, h(x_k)$ są $k$-niezależne
- **(jednostajność)** $\mathcal{H}$ jest jednostajna.

Zauważmy, że $k$-niezależność implikuje  $k$-uniwersalność (zachęcamy Czytelnika do zweryfikowania tego faktu).

Podamy teraz konkretne przykłady rodzin $(\alpha,k)$-uniwersalnych i $k$-niezależnych. Zauważmy, że rozważana w przykładzie 1 rodzina wszystkich funkcji jest $k$-uniwersalna, a nawet $k$-niezależna dla dowolnego $k$. Przykłady, które za chwilę poznamy, będą jednak bardziej praktyczne, w szczególności będą umożliwiały szybkie wylosowanie funkcji, oraz szybkie obliczenie jej wartości w danym elemencie $U$.

Haszowanie tabulacyjne  

W tym rozdziale rozważamy bardzo praktyczną sytuację.  Niech $U = \{0, \ldots, 2^{cr} - 1\}$, dla pewnych $c,r\in \mathbb{N}$, $c>1$.  Innymi słowy, uniwersum $U$ składa się z liczb $(cr)$-bitowych. Każdą taką liczbę $x$ będziemy reprezentować jako krotkę $(x_1, x_2, \ldots, x_c)$ liczb r-bitowych. W typowych zastosowaniach $U$ składa się z 32- lub 64-bitowych liczb naturalnych, natomiast $r=8$, czyli krotka to kolejne bajty liczby $x$.

Ponadto zakładamy, że długość tablicy haszującej $m$ jest potęgą dwójki, $m = 2^k$. To założenie nie jest zbyt restrykcyjne, gdyż w najgorszym razie tablica haszująca będzie mniej niż 2 razy większa niż potrzeba.

Oznaczmy $[\ell] = \{0,\ldots, \ell-1\}$. Rozważmy następującą rodzinę funkcji haszujących postaci $h:U\rightarrow [m]$:

$$
\mathcal{H} = \{ x \mapsto T_1 (x_1) \oplus T_2 (x_2) \oplus \ldots \oplus T_c (x_c)\ \vert{}\ T_1,...,T_c : [2^r] \rightarrow [m] \},
$$

gdzie $\oplus$ oznacza operację xor, np $6 \oplus 3 = (110)_2 \oplus (011)_2 = (101)_2 = 5.$

**Przykład 3**  

 Załóżmy że $U=\{0,\ldots,2^{32}-1\}$ oraz $m=2^{16}=65536$. Wybierzmy $c=4$ (wówczas $r=8$). Wówczas każda funkcja $h$ z rodziny $\mathcal{H}$ jest reprezentowana za pomocą czterech tablic $T_1, T_2, T_3, T_4$, z których każda zawiera $2^8=256$ liczb z przedziału $\{0,\ldots,m-1\}$. Sumarycznie, do reprezentowania $h$ potrzebujemy $4\cdot 256\cdot 2=1024$ bajty. Aby obliczyć wartość $h(x)$ dla danego $x$, wykonywane są 3 operacje $\oplus$ na 4 liczbach odczytanych z tablic. Zajmuje to czas stały, ale warto dodatkowo zwrócić uwagę, że wykonywane operacje są bardzo proste, więc obliczenie wartości $h(x)$ będzie bardzo szybkie (dodatkowo część obliczeń zostanie zrównoleglonych na współczesnych procesorach). Aby wylosować funkcję z rodziny $\mathcal{H}$, losujemy 512 liczb z zakresu $\{0,\ldots, m\}$ i umieszczamy w tablicach $T_1,\ldots,T_4$.

**Twierdzenie 4**  

Rodzina $\mathcal{H}$ jest $2$-niezależna.

*Dowód*  

Na początek sprawdźmy, że $\mathcal{H}$ jest jednostajna, tzn.\ dla każdego $x\in U$ i $a\in [m]$, $\mathbb{P}[h(x)=a]=\frac{1}{m}$. Jest to prawda, gdyż po wylosowaniu $T_1(x_1),\ldots,T_{c-1}(x_{c-1})$, niezależnie od wyników tych losowań, otrzymamy $h(x)=a$ gdy $T_c(x_c)=a\oplus T_1 (x_1) \oplus T_2 (x_2) \oplus \ldots \oplus T_{c-1} (x_{c-1})$, a to zdarzenie ma prawdopodobieństwo $\frac{1}{m}$. (Formalnie możemy przeprowadzić to rozumowanie korzystając ze wzrou na prawdopodobieństwo całkowite.)

Rozważmy teraz dwa różne elemety uniwersum $x,y \in U$, oraz dowolne $a,b\in[m]$. Z powodu symetrii, bez straty ogólności możemy przyjąć, że $x_1\ne y_1$. Po wylosowaniu $T_2(x_2),\ldots,T_c(x_c)$ oraz $T_2(y_2),\ldots,T_c(y_c)$, niezależnie od wyników tych losowań, otrzymamy $h(x)=a$ i $h(y)=b$ wtw gdy $T_1(x_1)=a\oplus T_2(x_2)\oplus\ldots\oplus T_c(x_c)$ oraz $T_1(y_1)=b\oplus T_2(y_2)\oplus\ldots\oplus T_c(y_c)$. Ponieważ $x_1\ne y_1$ te dwa zdarzenia są niezależne, czyli:

$$
\begin{align}  

 & \mathbb{P}[h(x)=a \wedge h(y)=b]  =\\  

 & \mathbb{P}[T_1(x_1)=a\oplus T_2(x_2)\oplus\ldots\oplus T_c(x_c) \wedge T_1(y_1)=b\oplus T_2(y_2)\oplus\ldots\oplus T_c(y_c)]=\\  

                           & \mathbb{P}[T_1(x_1)=a\oplus T_2(x_2)\oplus\ldots\oplus T_c(x_c)] \cdot \mathbb{P}[T_1(y_1)=b\oplus T_2(y_2)\oplus\ldots\oplus T_c(y_c)]=\\  

                           & \frac{1}{m}\cdot\frac{1}{m} = \frac{1}{m^2}.  

\end{align}
$$  

♦

**Zadanie**  

 Udowodnij, że rodzina $\mathcal{H}$ jest $3$-niezależna dla dowolnego $c\ge 1$, ale nie jest $4$-niezależna gdy $c\ge 2$.

  

Rodzina funkcji liniowych  

Niech $p$ będzie dowolną liczbą pierwszą większą od $\vert{}U\vert{}$. Rozważmy następującą rodzinę.  

$$  

\mathcal{H} = \{ x \mapsto \left( (ax+b)\bmod p\right)\bmod m\ |\ a \in \{0,\ldots,p-1\}, b \in \{0,\ldots,p-1\}\}.  

$$

Zauważmy, że każda funkcja z $\mathcal{H}$ jest reprezentowana przez zaledwie dwie liczby $O(\log\vert{}U\vert{})$-bitowe $a$ i $b$. Z praktycznego punktu widzenia wygląda to bardziej atrakcyjnie niż reprezentacje funkcji z rodzin tabulacyjnych (np. 1024 bajty w funkcji z przykładu 3). Ta rodzina ma jednak znacznie mniejsze znaczenie praktyczne ze względu na czas obliczania wartości funkcji. Chociaż pozostaje on stały w sensie asymptotycznym, podczas obliczania wartości funkcji wykonywane są kosztowne operacje mnożenia i obliczania reszty z dzielenia przez $p$ i przez $m$ (można ten efekt ograniczyć, przyjmując jako $m$ potęgę dwójki, natomiast jako $p$ wziąć liczbę pierwszą szczególnej postaci, mimo wszystko eksperymenty pokazują jednak, że obliczanie wartości w haszowaniu tabulacyjnym jest istotnie szybsze).

**Twierdzenie 5**  

Rodzina $\mathcal{H}$ jest $2$-uniwersalna.

*Dowód*  

Weźmy $x_1 \neq x_2$ należące do $U$. Oznaczmy  

$$
\begin{align*}  

x_1' &:= (ax_1 + b) \bmod p\\  

x_2' &:= (ax_2 + b) \bmod p  

\end{align*}
$$  

Dzięki temu, że $p$ jest pierwsza, $\mathbb{Z}_p$ jest ciałem, a w ciele:  

$$  

ax_1+b = ax_2 + b \iff x_1 = x_2,  

$$  

czyli $x_1' \neq x_2'$.

Pokażemy teraz, że dla dowolnych $i \neq j$ ze zbioru $\{0,\ldots,p-1\}$ zachodzi (*)  

$$
\mathbb{P}[x_1' = i \wedge x_2' = j] = \frac{1}{p(p-1)},
$$  

a więc, że para $(x_1',x_2')$ jest losową parą uporządkowaną różnych liczb z $\mathbb{Z}_p$.

Zbiór funkcji haszujących $\mathcal{H}$ rozmiaru $p(p-1)$ jest naszą przestrzenią probabilistyczną. Ile jest zdarzeń elementarnych (funkcji haszujących $h \in \mathcal{H}$, czyli par $(a,b)$), takich że $x_1'=i$ oraz $x_2'=j$? Każda taka para $(a,b)$ jest wyznaczona przez układ równań  

$$
\left\{ \begin{array}{ll} ax_1 + b = i \\ ax_2+b = j.\end{array} \right.
$$  

Ten układ ma jednoznaczne rozwiązanie, ponieważ  

$$
\det\left[ \begin{array}{ll} x_1 & 1 \\ x_2 & 1\end{array} \right] \neq 0.
$$  

Zatem istnieje dokładnie jedna para spośród $p(p-1)$, która spełnia układ równań, zatem (*) jest udowodnione.

Teraz pokażemy, że (**)  

$$
\mathbb{P}[x_1' \equiv x_2' \pmod m] \leq \frac{1}m.
$$  

Jeśli to zdarzenie zachodzi, to dla pewnych $k,l$  

$$
\left\{ \begin{array}{ll} x_1' = km + r \\ x_2' = lm + r.\end{array} \right.
$$  

Dla ustalonego $x_1'$ istnieje co najwyżej $\lceil\frac{p}m\rceil -1$ liczb $l\ne k$, które dadzą nam taki $x_2'$ (odejmujemy 1 bo wiemy, że $x_1'\ne x_2'$), zatem sumując po wszystkich $p$ możliwych wartościach $x_1'$ dostajemy  

$$
\mathbb{P}[x_1' \equiv x_2' \pmod m] \leq p \frac{\lceil\frac{p}m\rceil-1}{p(p-1)} \leq  \frac{\frac{p+m-1}m-1}{p-1} = \frac{p-1} {m(p-1)} = \frac{1}m
$$  

Z (**) wynika, że rodzina~$\mathcal{H}$ jest rzeczywiście 2-uniwersalna. ♦

Można pokazać (do czego zachęcamy czytelnika), że rodzina $\mathcal{H}$ nie jest 2-niezależna.

  

Rodzina wielomianów  

W tym punkcie rozważamy uogólnienie rodziny z poprzedniego punktu.  

$$  

\mathcal{H}^m = \{ x \mapsto \left( (a_0 + a_1x + \ldots + a_{k-1}x^{k-1})\bmod p\right)\bmod m\ |\ a_i \in \{0,\ldots,p-1\}\}.  

$$

Zauważmy, że tym razem każdy ze współczynników $a_1,\ldots,a_{k-1}$ może być równy 0, a więc $\mathcal{H}^m$ zawiera funkcje stałe, które w kontekście zastosowań słownikowych zachowują się fatalnie! Taka definicja pozwala jednak uprościć rozumowania (zauważmy ponadto, że prawdopodobieństwo wylosowania funkcji stałej jest bardzo małe).

Pokażemy, że $\mathcal{H}^m$ jest $(O(1),k)$-uniwersalna, a nawet że bardzo niewiele brakuje jej, aby być $k$-niezależną (spełniony jest warunek niezależności zmiennych, natomiast rozkład zmiennej $h(x)$ jest *niemal* jednostajny), szczególnie jeśli $p \gg m$.

Rozważmy parami różne zmienne $x_1, \ldots, x_k$. Podobnie jak poprzednio niech $x_i':= \sum_{j} a_jx_i^j \bmod p$ (a więc $h(x_i) = x_i' \bmod m$).

Pokażemy najpierw, że dla dowolnych $y'_1,\ldots,y'_k \in \{0,\ldots,p-1\}$ zachodzi (***)  

$$
\mathbb{P}[\bigcap_{i=1}^k x_i' = y'_i] = \frac{1}{p^k}.
$$  

To zdarzenie odpowiada układowi równań w ciele $\mathbb{Z}_p$:  

$$
\left\{ \begin{array}{ccl}  

a_0 + a_1x_1 + \ldots + a_{k-1}x_1^{k-1}& \equiv &y'_1 \pmod p\\  

a_0 + a_1x_2 + \ldots + a_{k-1}x_2^{k-1}& \equiv &y'_2 \pmod p\\  

\vdots&\vdots&\\  

a_0 + a_1x_k + \ldots + a_{k-1}x_k^{k-1}& \equiv &y'_k \pmod p  

\end{array} \right.
$$  

Macierz  

$$
\left[ \begin{array}{cccc}  

1 & x_1 & \ldots & x_1^{k-1}  \\  

1 & x_2 & \ldots & x_2^{k-1}  \\  

\vdots&\vdots& &\vdots\\  

1 & x_k & \ldots & x_k^{k-1}  \\  

\end{array} \right]
$$  

tego układu jest [macierzą Vandermonde'a](http://pl.wikipedia.org/wiki/Macierz_Vandermonde%27a), a ta ma niezerowy wyznacznik dla parami różnych $x_1,\ldots,x_k$. Zatem istnieje dokładnie jedno rozwiązanie $(a_0,\ldots,a_{k-1})$ tego układu, a więc równość (***) jest udowodniona.

W tej chwili zauważmy, że właśnie pokazaliśmy następujący wniosek.

**Wniosek 6**  

Rodzina $\mathcal{H}^p = \{ x \mapsto  (a_0 + a_1x + \ldots + a_{k-1}x^{k-1})\bmod p\ \vert{}\ a_i \in \{0,\ldots,p-1\}\}$ jest rodziną $k$-niezależną.

W szczególności powyższy fakt implikuje, że jeśli $h$ wylosowano z $\mathcal{H}^p$ to zmienne losowe $h(x)$ dla $x\in U$ są $k$-niezależne. Z tego łatwo wynika (zachęcamy czytelnika do sprawdzenia), że:

**Wniosek 7**  

Jeśli $h$ wylosowano z $\mathcal{H}^m$ to zmienne losowe $h(x)$ dla $x\in U$ są $k$-niezależne.

Aby *rodzina* $\mathcal{H}^m$ była $k$-niezależna, potrzeba jednak jeszcze, żeby funkcja $h$ była jednostajna, a to nie do końca jest prawdą.

Analogicznie jak w poprzednim punkcie, dla ustalonych $y_1,\ldots,y_k \in \{0,\ldots,m-1\}$ mamy  

$$
\mathbb{P}[\bigcap_{i=1}^k h(x_i) = y_i] = \mathbb{P}[\bigcap_{i=1}^k x'_i \equiv y_i \pmod m] \leq {\left\lceil \frac{p}m\right\rceil}^k \frac{1}{p^k},
$$  

ponieważ dla każdego $y_i$ istnieje co najwyżej $\lceil \frac{p}m\rceil$ wartości $x'_i$, że $x'_i \equiv y_i \mod m$, a każda konkretna krotka jest losowana z prawdopodobieństwem $\frac{1}{p^k}$ na mocy równości (***).

Widzimy, że nie dostaliśmy tu oszacowania przez $\frac{1}{m^k}$, którego wymaga defnicja rodziny $k$-niezależnej, jednakże zwykle $m \ll p$ a więc ${\left\lceil \frac{p}m\right\rceil}^k \frac{1}{p^k}$ jest bardzo bliskie $\frac{1}{m^k}$. Jeśli np. dobierzemy $p$ tak, aby $\frac{m-1}{p} \leq \frac{1}k$, to  

$$  

{\left\lceil \frac{p}m\right\rceil}^k \frac{1}{p^k} \leq \left( \frac{p+m-1}{pm}\right)^k =  \left( \frac{1+\frac{m-1}p}{m}\right)^k < \frac{e}{m^k}.  

$$  

Wtedy $\mathcal{H}^m$ jest $(e,k)$-uniwersalna.

  

Bardziej praktyczna rodzina funkcji liniowych  

W praktyce rozmiar uniwersum i $m$ są potęgami dwójki: $U = \{0,\ldots,2^k-1\}$, $m=2^l$. Wtedy $h: \{0,\ldots,2^k-1\} \longrightarrow \{0,\ldots,2^l-1\}$. Dietzfelbinger zaproponował następującą rodzinę funkcji haszujących:  

$$  

\mathcal{H}_{k,l} = \{ x \mapsto (ax \bmod 2^k) \mbox{ div } 2^{k-l}\ |\ a \in \{0,\ldots,2^k-1\} \wedge a \mbox{ nieparzyste} \}.  

$$  

Operacja $\mbox{div }2^{k-l}$ bierze $l$ pierwszych (najbardziej znaczących) bitów. Implementacja funkcji z powyższej rodziny jest bardzo łatwa, gdy liczby typu `int` są z przedziału $\{0,\ldots,2^k-1\}$. Odpowiedni kod w języku C wyglądałby następująco:

```
int function h(int x) {
  return (a*x) >> (k-l);
}
```

Funkcje te są często stosowane w praktyce, ponieważ obliczanie ich wartości jest bardzo szybkie (znaczenie szybsze niż w przypadku funkcji liniowych opisanych wcześniej, choć również istotnie wolniejsze niż w przypadku funkcji z rodziny tabulacyjnej), natomiast same funkcje są bardzo proste w implementacji.

**Twierdzenie 8**  

Rodzina $\mathcal{H}_{k,l}$ jest $(2,2)$-uniwersalna.

*Dowód*  

Niech $x,y \in \{0,\ldots,2^k-1\}$. Załóżmy, że $x > y$ i niech $h_a$ będzie funkcją wybraną losowo z $\mathcal{H}_{k,l}$. Chcemy pokazać, że  

$$
\mathbb{P}[h_a(x) = h_a(y)] \leq \frac{1}{2^{l-1}}.
$$  

Policzmy, ile jest takich $a$, dla których $h_a(x) = h_a(y)$. Ta równość jest równoważna nierówności  

$$
\vert{} ax \bmod 2^k - ay \bmod 2^k\vert{} < 2^{k-l}.
$$  

Niech $z = x - y$, wtedy powyższą nierówność możemy zapisać jako (#)  

$$
\vert{} az \bmod 2^k \vert{} < 2^{k-l}.
$$  

Z założenia $z \not\equiv 0 \pmod{2^k}$ oraz $a$ jest nieparzyste, zatem (##)  

$$
az \not\equiv 0 \pmod {2^k}
$$  

Warunki (#) i (##) zachodzą, gdy (###)  

$$
az \bmod 2^k \in \{1,\ldots,2^{k-l}-1\} \cup \{2^k - 2^l+1, \ldots, 2^k-1\}
$$  

Niech $z = 2^s\cdot z'$, gdzie $z'$ jest nieparzyste. Zbiór $A = \{ 1, 3, 5, \ldots, 2^k-1\}$ jest grupą z mnożeniem modulo ${2^k}$. Wówczas $\{z'\cdot a\ \vert{}\ a \in A\} = A$, a więc ilość liczb $a \in A$ spełniających (###) jest równa ilości liczb $a$, dla których  

$$
a\cdot 2^s \bmod 2^k\in \{1,\ldots,2^{k-l}-1\} \cup \{2^k - 2^l+1, \ldots, 2^k-1\}.
$$  

Pierwszy zbiór jest postaci $\underbrace{0\ldots 0}_{l \text{ bitów}}\underbrace{\underline{\text{coś}\;\neq 0}}_{k-l\text{ bitów}}$ lub $\underbrace{1\ldots 1}_{l \text{ bitów}}\underbrace{\underline{\text{coś}\;\neq 0}}_{k-l\text{ bitów}}$. Jeśli $s \geq k-l$, to końcówka będzie zerowa, więc nie ma takich liczb $a$. Jeśli $s < k-l$, to $a$ zaczyna się od samych $1$ lub samych $0$, potem wybieramy $k-l$ bitów, z których ostatni musi być równy $1$, zatem $a$ można wybrać na $2\cdot 2^{k-l-1} = 2^{k-l}$ sposobów, co daje ostatecznie, że $\mathbb{P}[h_a(x) = h_a(y)] \leq \frac{2^{k-l}}{2^{k-1}} = \frac{1}{2^{l-1}}=\frac{2}{m}.$ Stąd, rodzina $\mathcal{H}_{k,l}$ jest $(2,2)$-uniwersalna. ♦
