---
layout: 'default'
title: 'Programowanie liniowe 4: Programy całkowitoliczbowe i kilka zastosowań dualności'
page_url: 'http://smurf.mimuw.edu.pl/node/1126'
source_url: 'http://smurf.mimuw.edu.pl/node/1126'
source_kind: 'html'
uses_math: true
render_with_liquid: false
rendered_file: 'lectures/zaawansowane-algorytmy-i-struktury-danych/programowanie-liniowe-4-programy-cakowitoliczbowe-i-kilka-zastosowan-dualnosci/rendered.html'
---

# Programowanie liniowe 4: Programy całkowitoliczbowe i kilka zastosowań dualności

## Programowanie całkowitoliczbowe

Program (liniowy) całkowitoliczbowy to program liniowy z dodatkowym wymaganiem, aby wartości wszystkich zmiennych były całkowitoliczbowe. Ten z pozoru niewinny warunek całkowicie zmienia złożoność problemu: większość naturalnych problemów NP-trudnych można bardzo łatwo wyrazić jako liniowe programy całkowitoliczbowe, np. problem pokrycia wierzchołkowego w grafie $G=(V,E)$ jest równoważny programowi:

$$
\label{eq:vc}  

       \begin{array}{rll}  

       \textrm{min}                    & \sum_{v\in V} x_v \\  

                                       & x_u+x_v \ge 1 & \forall uv \in E\\  

                                       & x_v \in \{0,1\} & \forall v \in V,\\  

       \end{array}
$$  

ponieważ ostatni warunek możemy zastąpić przez koniunkcję $0 \le x_v \le 1$ i $x_v \in \mathbb{Z}$. Jeśli min zastąpimy przez max, a $\ge$ przez $\le$ otrzymamy z kolei problem najmniejszego zbioru niezależnego.

**Wniosek**  

 Problem programowania liniowego całkowitoliczbowego jest NP-trudny.

## Dokładne relaksacje i unimodularność

Dla danego programu całkowitoliczbowego (I) jego *relaksacją* będziemy nazywać program liniowy (L), który powstaje po usunięciu warunku całkowitoliczbowości. Relaksacja jest *dokładna*, gdy wszystkie wierzchołki (L) są całkowitoliczbowe (tzn. mają całkowite współrzędne). Zauważmy, że jeśli relaksacja faktycznie jest dokładna, to w szczególności możemy rozwiązać program całkowitoliczbowy (I) w czasie wielomianowym. W tym celu wystarczy:

1. rozwiązać (L) w czasie wielomianowym (np. metodą elipsoidalną) otrzymując rozwiązanie optymalne $\mathbf{x}^*$,
2. znaleźć wierzchołek $\mathbf{x}'$ o wartości funkcji celu takiej samej jak dla $\mathbf{x}^*$ (wielomianowo, np. tak jak w dowodzie twierdzenia th:opt->vtx z [wykładu o geometrii programów liniowych](http://smurf.mimuw.edu.pl/drupal6/node/1121))
3. zwrócić $\mathbf{x}'$ jako rozwiązanie programu całkowitoliczbowego (I).

Poszukiwanie programów całkowitoliczbowych, których relaksacje są dokładne to użyteczne narzędzie w rozstrzyganiu, czy dany problem algorytmiczny należy do klasy $\mathbf{P}$. W dowodzeniu, że dana relaksacja jest dokładna często używamy pojęcia całkowitej unimodularności.

Mówimy, że macierz $\mathbf{A}$ jest *całkowicie unimodularna* jeśli dla każdej podmacierzy $\mathbf{A}'$ macierzy $\mathbf{A}$, mamy $\det \mathbf{A}'\in \{-1,0,1\}$.

**Twierdzenie 1**  

Jeśli wektor $\mathbf{b}$ jest całkowitoliczbowy i macierz $\mathbf{A}$ jest całkowicie unimodularna to program $\mathbf{A} \mathbf{x}\le \mathbf{b}$ ma wierzchołki całkowitoliczbowe (czyli jest dokładną relaksacją programu całkowitoliczbowego $\mathbf{A} \mathbf{x}\le \mathbf{b}$).

Dowód powyższego twierdzenia pozostawiamy Czytelnikowi jako ćwiczenie. Wskazówka: skorzystaj z tego, że wierzchołki są bazowymi rozwiązaniami dopuszczalnymi oraz użyj wzorów Cramera.

## Maksymalny przepływ o minimalnym koszcie

Rozważmy problem maksymalnego przepływu o minimalnym koszcie (patrz odpowiedni [wykład](http://smurf.mimuw.edu.pl/drupal6/node/1119)). Poznaliśmy dwa algorytmy dla tego problemu (algorytm usuwania ujemnych cykli i algorytm najkrótszej ścieżki). Choć algorytm najkrótszej ścieżki jest bardzo efektywny w zastosowaniach, w których maksymalny przepływ ma niewielką wartość, w ogólności żaden z tych dwóch algorytmów nie jest wielomianowy (tzn. nie zależy wielomianowo od rozmiaru danych). Z pomocą programowania liniowego łatwo pokazać, że algorytm wielomianowy dla tego problemu istnieje, i to nawet w ogólnym przypadku (algorytm najkrótszej ścieżki działa tylko gdy w grafie wejściowym nie ma cykli o ujemnym koszcie). Wystarczy jednym z algorytmów wielomianowych rozwiązać następujący program liniowy:

Program (F)  

$$
\begin{array}{ll@{\hspace{7mm}}l}  

       \textrm{zminimalizuj}           & \sum_{(u,v)\in E} a(u,v) \cdot f_{uv} & \\  

       \textrm{z zachowaniem warunków} &  \sum_{(u,v)\in  E} f_{uv} - \sum_{(v,u)\in E}f_{vu} = 0 & \forall v \in V\setminus\{s,t\} \\  

                                       & f_{vw} \le c(v,w) & \forall (v,w)\in E \\  

                                       & f_{vw} \ge 0 & \forall (v,w)\in E \\  

                                       & \sum_{(s,v)\in E}f_{sv}=f^*,  

       \end{array}
$$

gdzie $f^*$ jest wartością maksymalnego przepływu (którą możemy wyliczyć w czasie wielomianowym algorytmem Edmondsa-Karpa lub rozwiązując program liniowy z przykładu 2 z [pierwszego wykładu o programowaniu liniowym](http://smurf.mimuw.edu.pl/drupal6/node/1121)).

Znane nam algorytmy dla problemów przepływowych mają tę użyteczną cechę, że gdy przepustowości w sieci są całkowitoliczbowe, to również przepływ znajdowany przez te algorytmy jest całkowitoliczbowy. Okazuje się, że można to zagwarantować również rozwiązując powyższy program, gdyż jego macierz jest całkowicie unimodularna. Poniżej przestawiamy dowód tego faktu.

**Twierdzenie 2**  

Program (F) ma wierzchołki całkowitoliczbowe, o ile przepustowości krawędzi są liczbami całkowitymi.

*Dowód*  

Pokażemy, że macierz programu (F) jest unimodularna. Rozważamy dowolną podmacierz $A'$ i chcemy wykazać, że $\det A'\in \{-1,0,1\}$.  Dowodzimy przez indukcję ze względu na liczbę elementów macierzy $A'$, że $\det A' \in \{-1,0,1\}$. Dla macierzy o wymiarach $1\times 1$ mamy $\det A' \in \{-1,0,1\}$. Rozważmy teraz podmacierz $A'$ o dowolnych wymiarach. Jeśli $A'$ ma kolumnę, która zawiera co najwyżej jedną 1-kę, kończymy dowód korzystając z rozwinięcia Laplace'a względem tej kolumny i założenia indukcyjnego. Postępujemy analogicznie dla wierszy. Został nam przypadek, gdy każdy wiersz i każda kolumna zawiera co najmniej dwie 1-ki. Stąd, każdy wiersz $A'$ pochodzi od ograniczenia typu $\sum_{(u,v)\in  E} f_{uv} - \sum_{(v,u)\in E}f_{vu} = 0$ lub od ograniczenia $\sum_{(s,v)\in E}f_{sv}=f^*$. Ale dla każdej zmiennej $f_{ab}$ są tylko dwa takie ograniczenia: w jednym (dla wierzchołka $a$)  $f_{ab}$ pojawia się ze współczynnikiem $+1$, w drugim (dla wierzchołka $b$) ze współczynnikiem $-1$. Po dodaniu wszystkich wierszy $A'$ dostaniemy więc wektor zerowy, a więc wiersze $A'$ są liniowo zależne i $\det A'=0$. ♦

## Skojarzenia w grafach dwudzielnych i twierdzenie Koniga-Egervary'ego

Rozważmy problem maksymalnego skojarzenia w danym grafie $G=(V,E)$. Problem ten możemy łatwo sformułować jako zadanie programowania liniowego całkowitoliczbowego.

Program (IM1)  

$$
\label{eq:matching}  

       \begin{array}{rll}  

       \textrm{max}                    & \sum_{e \in E} x_e \\  

                                       & \sum_{vw\in E}x_{vw} \le 1 & \forall v \in V\\  

                                       & x_e \in \{0,1\} &  \forall  e \in E.\\  

       \end{array}
$$

Relaksacja tego programu wygląda następująco:

Program (M1)  

$$
\label{eq:matching-lp}  

       \begin{array}{rll}  

       \textrm{max}                    & \sum_{e \in E} x_e \\  

                                       & \sum_{vw\in E}x_{vw} \le 1 & \forall v \in V\\  

                                       & x_e \ge 0 & \forall e \in E.\\  

       \end{array}
$$

Zauważmy, że w powyższym programie mogliśmy opuścić warunek $x_e \le 1$, gdyż i tak jest on spełniony dla każdego rozwiązania dopuszczalnego programu (M1).

**Twierdzenie 3**  

 Jeśli $G$ jest dwudzielny, to macierz programu (M1) jest unimodularna.

*Dowód*  

Ponieważ $G$ jest dwudzielny, więc jego zbiór wierzchołków dzieli się na dwa rozłączne zbiory $V=X\cup Y$ takie, że każda krawędź ma 1 koniec w $X$ i 1 koniec w $Y$. Niech $A'$ będzie dowolną podmacierzą macierzy programu (M1).  Stosujemy indukcję podobnie jak w dowodzie twierdzenia 2. Analogicznie jak poprzednio możemy założyć, że w każdym wierszu i każdej kolumnie $A'$ są co najmniej dwie 1-ki. Stąd każdy wiersz $A'$ pochodzi od wiersza typu $\sum_{vw\in E}x_{vw} \le 1$ macierzy $A$. Zauważamy, że każda kolumna macierzy $A'$ zawiera dokładnie dwie jedynki: jedną w pewnym wierszu odpowiadającemu wierzchołkowi z $X$, drugą w wierszu odpowiadającemu wierzchołkowi z $Y$. Stąd, po dodaniu wszystkich wierszy $A'$ odpowiadających wierzchołkom z $X$ dostaniemy taki sam wektor jak po dodaniu wszystkich wierszy  $A'$ odpowiadających wierzchołkom z $Y$. A więc wiersze macierzy $A'$ są liniowo zależne, czyli $\det A'=0$.  ♦

Jako wniosek dostajemy, że (M1) jest dokładną relaksacją (IM1). W konsekwencji mamy

**Wniosek**  

Problem największego skojarzenia w grafach dwudzielnych jest w klasie $\mathbf{P}$.

Ten wniosek można oczywiście również otrzymać podając konkretny algorytm wielomianowy (patrz np. [wykład o skojarzeniach w grafach dwudzielnych](http://wazniak.mimuw.edu.pl/index.php?title=Zaawansowane_algorytmy_i_struktury_danych%2FWyk%C5%82ad_7)).

Na marginesie dodajmy, że  (M1) **nie** jest dokładną relaksacją (IM1) w klasie wszystkich grafów (łatwo to sprawdzić np. dla trójkąta). Aby otrzymać program, którego relaksacja jest dokładna, należy do (M1) dodać *wykładniczą liczbę* warunków: dla każdego nieparzystego zbioru wierzchołków $S\subseteq V$ dodajemy warunek $\sum_{e=vw: v,w\in S} x_e \le (\vert{}S\vert{}-1)/2$.

Powróćmy do grafów dwudzielnych. Rozważmy program dualny do programu (M1):

Program (VC)  

$$
\label{eq:vc-2}  

       \begin{array}{rll}  

       \textrm{min}                    & \sum_{v\in V} y_v \\  

                                       & y_u+y_v \ge 1 & \forall uv \in E\\  

                                       & y_v \ge 0 & \forall v \in V.\\  

       \end{array}
$$

W powyższym programie moglibyśmy dodać warunek $y_v \le 1$: i tak jest on spełniony dla każdego rozwiązania optymalnego programu (VC). Ponieważ macierz programu (VC) jest transpozycją macierzy programu (M1), więc również jest unimodularna. Stąd program (VC) ma rozwiązanie optymalne, które jest również rozwiązaniem optymalnym następującego programu całkowitoliczbowego

Program (IVC)  

$$
\label{eq:vc-3}  

       \begin{array}{rll}  

       \textrm{min}                    & \sum_{v\in V} y_v \\  

                                       & y_u+y_v \ge 1 & \forall uv \in E\\  

                                       & y_v \in \{0,1\} & \forall v \in V.\\  

       \end{array}
$$

Z kolei zauważamy, że powyższy program jest równoważny problemowi znalezienia najmniejszego pokrycia wierzchołkowego grafu $G$. Na podstawie twierdzenia o silnej dualności otrzymujemy

**Twierdzenie [Kőnig, Egervary]**  

W grafie dwudzielnym rozmiar największego skojarzenia jest równy liczności najmniejszego pokrycia wierzchołkowego. ♦

Wiele innych klasycznych twierdzeń mini-maksowych okazuje się być szczególnymi przypadkami twierdzenia o dualności programów liniowych.

## Lemat Farkasa

Pokażemy teraz zastosowanie dualności w algebrze liniowej. Przypomnijmy, że dla układów równań prawdziwa jest następująca elegancka własność:

**Lemat**  

Są równoważne:

1. istnieje $\mathbf{x}$ taki, że $\mathbf{A}\mathbf{x}=\mathbf{b}$,
2. nie istnieje $\mathbf{y}$ taki, że $\mathbf{y}^T\mathbf{A}=\mathbf{0}$ oraz $\mathbf{y}^T\mathbf{b}\ne\mathbf{0}$.

Innymi słowy, albo układ ma rozwiązanie, albo istnieje taka kombinacja liniowa $\mathbf{y}$ jego równań, która prowadzi do sprzeczności. Odpowiednik tego faktu dla programów liniowych nosi nazwę Lemat Farkasa.

**Lemat Farkasa**  

Są równoważne:

1. istnieje $\mathbf{x}$ taki, że $\mathbf{A}\mathbf{x}\le\mathbf{b}$,
2. nie istnieje $\mathbf{y}\ge\mathbf{0}$ taki, że $\mathbf{A}^T\mathbf{y}=\mathbf{0}$ oraz $\mathbf{y}^T\mathbf{b}<0$.

*Dowód*  

Rozważmy (prymalny) program liniowy:  

$$
\label{eq:st-primal-farkas}  

       \begin{array}{ll@{\hspace{15mm}}l}  

       \textrm{zmaksymalizuj}           & 0 &  \\  

       \textrm{z zachowaniem warunków} & \mathbf{A}\mathbf{x} \le \mathbf{b}&\\  

                                       & \mathbf{x} \text{ nieograniczony}&  

       \end{array}
$$  

Napiszmy program dualny zgodnie z zasadami z [wykładu o dualności](http://smurf.mimuw.edu.pl/drupal6/node/1124).  

$$
\label{eq:st-dual-farkas}  

       \begin{array}{ll@{\hspace{15mm}}l}  

       \textrm{zminimalizuj}           & \mathbf{b}^T \mathbf{y} &  \\  

       \textrm{z zachowaniem warunków} & \mathbf{A}^T\mathbf{y} = \mathbf{0}&\\  

                                       & \mathbf{y} \ge 0.&  

       \end{array}
$$

(i) $\Rightarrow$ (ii). Jeśli istnieje $\mathbf{x}$ taki, że $\mathbf{A}\mathbf{x}\le\mathbf{b}$, to program prymalny jest dopuszczalny i ma rozwiązanie optymalne o koszcie 0. Z twierdzenia o (słabej) dualności, program dualny ma rozwiązań dopuszczalnych o wartości funkcji celu $<0$.

(ii) $\Rightarrow$ (i). Jeśli nie istnieje $\mathbf{y}\ge\mathbf{0}$ taki, że $\mathbf{A}^T\mathbf{y}=\mathbf{0}$ oraz $\mathbf{y}^T\mathbf{b}<0$, oznacza to, że $\mathbf{y}=\mathbf{0}$ jest rozwiązaniem optymalnym programu dualnego. Z twierdzenia o (silnej) dualności program prymalny ma rozwiązanie optymalne, a więc układ nierówności $\mathbf{A}\mathbf{x} \le \mathbf{b}$ jest niesprzeczny. ♦
