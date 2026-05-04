---
layout: 'default'
title: 'Programowanie liniowe 1: Podstawowe pojęcia i geometria programów liniowych'
page_url: 'http://smurf.mimuw.edu.pl/node/1121'
source_url: 'http://smurf.mimuw.edu.pl/node/1121'
source_kind: 'html'
uses_math: true
render_with_liquid: false
rendered_file: 'lectures/zaawansowane-algorytmy-i-struktury-danych/programowanie-liniowe-1-podstawowe-pojecia-i-geometria-programow-liniowych/rendered.html'
---

# Programowanie liniowe 1: Podstawowe pojęcia i geometria programów liniowych

## Wstęp i podstawowe pojęcia

### Co to jest programowanie liniowe?

*Program liniowy* (w skrócie PL) jest to problem minimalizacji/maksymalizacji liniowej *funkcji celu* o $n$ argumentach $x_1, x_2, \ldots, x_n$ przy zachowaniu pewnej liczby równości lub nierówności liniowych (będziemy je nazywać *ograniczeniami*) zawierających zmienne $x_i$.

Przykład 1: prosty program liniowy  

$\begin{array}{lr}  

       \text{zminimalizuj}           & x_1 + 2x_2  \\  

       \text{z zachowaniem warunków} & x_2 \le x_1 + 2 \\  

                                       & 2x_1+x_2\le 4\\  

                                       & 2x_2+x_1\ge 0\\  

                                       & x_2 \ge 0  

       \end{array}$

Za pomocą programów liniowych można wyrazić bardzo wiele naturalnych problemów optymalizacyjnych. Dla przykładu, problem maksymalnego przepływu w sieci o $n$ wierzchołkach, źródle $s$, ujściu $t$, i funkcji przepustowości $c:V^2\rightarrow \mathbb{R}$ można wyrazić za pomocą następującego programu liniowego o $\vert{}V\vert{}^2$ zmiennych (dla każdej pary wierzchołków $v,w$ mamy zmienne $f_{vw}$ i $f_{wv}$ odpowiadające przepływowi od $v$ do $w$ i odwrotnie) i $2\vert{}V\vert{}^2+\vert{}V\vert{}-2$ ograniczeniach:

Przykład 2: program liniowy dla problemu maksymalnego przepływu  

$$
\begin{array}{ll@{\hspace{25mm}}l}  

       \textrm{zmaksymalizuj}          & \sum_{v\in V} f_{sv} & \\  

       \textrm{z zachowaniem warunków} & f_{vw} \le c(v,w) & \forall v,w\in V\\  

                                       & f_{vw} = -f_{wv} & \forall v,w\in V\\  

                                       & \sum_{w\in V} f_{vw} = 0 & \forall v\in V\setminus\{s,t\}\\  

       \end{array}
$$

Inny przykład: znajdowanie długości najkrótszej ścieżki od $s$ do $t$ w grafie $G=(V,E)$ z funkcją $w:E\rightarrow\mathbb{R}$ opisującą długości krawędzi. Tu dla każdego wierzchołka $v$ mamy zmienną $d_v$. Dla wierzchołków na najkrótszej ścieżce od $s$ do $t$ zmienna $d_v$ będzie odpowiadać odległości od $s$ do $v$.

$$
\begin{array}{ll@{\hspace{15mm}}l}  

       \textrm{zmaksymalizuj}          & d_t \\  

       \textrm{z zachowaniem warunków} & d_v \le d_u + w(u,v) & \forall (u,v)\in E\\  

                                       & d_s = 0 &  

       \end{array}
$$

**Zadanie.** Udowodnij, że ten program faktycznie jest równoważny problemowi najkrótszej ścieżki.

Rozwiązanie  

Dowód można przedstawić w następujący nieformalny sposób: wyobraźmy sobie że mamy fizyczny model grafu, w którym wierzchołki (powiedzmy metalowe kulki) są połączone sznurkami o odpowiednich długościach. Żeby znaleźć odległość od $s$ do $t$ chwytamy za kulki odpowiadające tym wierzchołkom i staramy się je maksymalnie od siebie oddalić (aż do całkowitego napięcia niektórych sznurków). W ten sposób otrzymujemy ,,ścieżkę napiętych krawędzi'' od $s$ do $t$. Gdyby istniała krótsza ścieżka, to nie bylibyśmy w stanie oddalić kulek tak bardzo.

### Terminologia i notacja

Rozważmy PL o $n$ zmiennych $x_1,\ldots,x_n$. Wartości zmiennych możemy utożsamiać z wektorem $\mathbf{x}=(x_1,\ldots,x_n)\in \mathbb{R}^n$. Zauważmy też, że liniową funkcję celu $\sum_{j=1}^n c_j x_j$ możemy zapisać krócej jako $\mathbf{c}^T\mathbf{x}$ dla wektora $\mathbf{c}=(c_1,\ldots,c_n)$. Będziemy utożsamiać tę funkcję z wektorem $\mathbf{c}$.

- $\mathbf{x}\in \mathbb{R}^n$ jest *rozwiązaniem dopuszczalnym* PL gdy $\mathbf{x}$ spełnia wszystkie ograniczenia.
- $\mathbf{x}\in \mathbb{R}^n$ jest *rozwiązaniem optymalnym* PL gdy $\mathbf{x}$ jest rozwiązaniem dopuszczalnym i optymalizuje funkcję celu, tzn. jeśli dla dowolnego dopuszczalnego $\mathbf{y}\in\mathbb{R}^n$ jest $\mathbf{c}^T\mathbf{x} \le \mathbf{c}^T\mathbf{y}$ w przypadku gdy PL jest minimalizacyjny ($\mathbf{c}^T\mathbf{x} \ge \mathbf{c}^T\mathbf{y}$ gdy maksymalizacyjny).
- PL jest *dopuszczalny* gdy istnieje rozwiązanie dopuszczalne, w przeciwnym przypadku jest *sprzeczny. - PL jest *nieograniczony* gdy jest dopuszczalny ale nie ma rozwiązań optymalnych, tzn. dla PL minimalizacyjnego dla dowolnego $\lambda \in \mathbb{R}$ istnieje rozwiązanie dopuszczalne $\mathbf{x}$ takie, że $\mathbf{c}^T\mathbf{x}<\lambda$.*

Będziemy posługiwać się trzema wygodnymi postaciami programów liniowych: kanoniczną, standardową i dopełnieniową.

#### Postać kanoniczna

Postać kanoniczna jest najbardziej ogólna z trzech rozważanych tu postaci. Program w postaci kanonicznej wygląda następująco:

$$
\begin{array}{ll@{\hspace{15mm}}l}  

       \textrm{zmaksymalizuj}           & \sum_{j=1}^n c_j x_j \\  

       \textrm{z zachowaniem warunków} & \sum_{j=1}^n a_{ij} x_j \le b_i & \forall i=1,\ldots,m  

       \end{array}
$$  

gdzie $\{a_{ij}, b_i, c_j\}$ są dane. Wygodniej będzie nam używać zapisu macierzowego:

$$
\begin{array}{ll@{\hspace{15mm}}l}  

       \textrm{zmaksymalizuj}           & \mathbf{c}^T\mathbf{x} &  \\  

       \textrm{z zachowaniem warunków} & \mathbf{A}\mathbf{x} \le \mathbf{b}&  

       \end{array}
$$

gdzie $\mathbf{c}\in\mathbb{R}^n, \mathbf{b}\in\mathbb{R}^m, \mathbf{A}\in M_{m\times n}[\mathbb{R}]$.

**Lemat 1**  

Każdy PL można sprowadzić do postaci kanonicznej.

*Dowód*  

Zadanie min $\mathbf{c}^T\mathbf{x}$ zamieniamy na max $(-\mathbf{c})^T\mathbf{x}$. Równości $\mathbf{a}_i^T\mathbf{x} = b_i$ zamieniamy na parę $\mathbf{a}_i^T\mathbf{x}\le b_i$, $\mathbf{a}_i^T\mathbf{x}\ge b_i$. Nierówności $\mathbf{a}_i^T\mathbf{x}\ge b_i$ zamieniamy na $(-\mathbf{a}_i)^T\mathbf{x}\le -b_i$. ♦

#### Postać standardowa

Programy liniowe modelujące problemy algorytmiczne bardzo często są w następującej postaci (jest to szczególny przypadek postaci kanonicznej):

$$
\begin{array}{ll@{\hspace{15mm}}l}  

       \textrm{zmaksymalizuj}           & \mathbf{c}^T\mathbf{x} &  \\  

       \textrm{z zachowaniem warunków} & \mathbf{A}\mathbf{x} \le \mathbf{b}&\\  

                                       & \mathbf{x} \ge \mathbf{0}&  

       \end{array}
$$

**Lemat 2**  

Każdy PL można sprowadzić do postaci standardowej.

*Dowód*  

Możemy założyć, że mamy już PL w postaci kanonicznej. Aby zapewnić nieujemność każdej zmiennej, dla każdej zmiennej $x_i$ wprowadzamy parę zmiennych $x_i^+$ i $x_i^-$ i każde wystąpienie $x_i$ w PL (także w funkcji celu) zastępujemy przez $x_i^+-x_i^-$. Dodajemy też $x_i^+\ge 0$ i $x_i^-\ge 0$. ♦

Czasem nie chcemy sztucznie zamieniać problemu minimalizacji na maksymalizację przez odwrócenie funkcji celu. Postać standardowa w wersji minimalizacyjnej wygląda następująco:

$$
\begin{array}{ll@{\hspace{15mm}}l}  

       \textrm{zminimalizuj}           & \mathbf{c}^T\mathbf{x} &  \\  

       \textrm{z zachowaniem warunków} & \mathbf{A}\mathbf{x} \ge \mathbf{b}&\\  

                                       & \mathbf{x} \ge \mathbf{0}&  

       \end{array}
$$

#### Postać dopełnieniowa

Postać dopełnieniowa jest najczęściej używana w algorytmach rozwiązujących problem programowania liniowego. Program w tej postaci (w wersji minimalizacyjnej) wygląda następująco:

$$
\begin{array}{ll@{\hspace{15mm}}l}  

       \textrm{zminimalizuj}           & \mathbf{c}^T\mathbf{x} &  \\  

       \textrm{z zachowaniem warunków} & \mathbf{A}\mathbf{x} = \mathbf{b}& \forall i=1,\ldots,m\\  

                                       & \mathbf{x} \ge \mathbf{0}  

       \end{array}
$$  

gdzie $\mathbf{c}\in\mathbb{R}^n, \mathbf{b}\in\mathbb{R}^m, \mathbf{A}\in M_{m\times n}[\mathbb{R}]$.

Zauważmy, że z dokładnością do zamiany równości na pary nierówności możemy powiedzieć, że PL w postaci dopełnieniowej jest w postaci standardowej.

**Lemat 3**  

Każdy PL można sprowadzić do postaci dopełnieniowej.

*Dowód*  

Możemy założyć, że mamy już PL w postaci standardowej. Dla każdej nierówności $\mathbf{a}_i^T\mathbf{x}\le b_i$ wprowadzamy ,,zmienną dopełnieniową'' $s_i$ i nierówność zastępujemy przez $\mathbf{a}_i^T\mathbf{x}-s_i=b_i$ oraz $s_i\ge 0$. ♦

## Geometria programów liniowych

Przypomnijmy:

- zbiór punktów spełniających $\mathbf{a}^T\mathbf{x}=b$, $\mathbf{a}\in\mathbb{R}^n, \mathbf{b}\in\mathbb{R}$ to *hiperpłaszczyzna*,
- zbiór punktów spełniających $\mathbf{a}^T\mathbf{x}\ge b$ to *półprzestrzeń*,
- zbiór rozwiązań dopuszczalnych PL w postaci kanonicznej $\mathbf{A}\mathbf{x}\le b$ to przecięcie półprzestrzeni, czyli *wielościan*.

Dla przykładu, zbiór rozwiązań dopuszczalnych PL z [Przykładu 1](http://smurf.mimuw.edu.pl/drupal6/node/1121) jest czworokątem (w 2 wymiarach hiperpłaszczyzny to proste, półprzestrzenie to półpłaszczyzny, a wielościany to wielokąty). Zauważmy, że rozwiązanie optymalne to najdalszy punkt wielościanu w kierunku wektora funkcji celu (dla problemu maksymalizacyjnego; dla minimalizacyjnego w kierunku wektora odwrotnego).

Zauważmy, że zbiór rozwiązań PL jest przecięciem zbiorów wypukłych (półprzestrzeni, hiperpłaszczyzn) a więc jest wypukły.

### Struktura rozwiązań optymalnych

Zdefiniujemy teraz trzy naturalne pojęcia związane z programami liniowymi i ich geometryczną interpretacją. Za chwilę pokażemy, że są one równoważne.

- *Wierzchołkiem* wielościanu $\mathcal{P}$ nazywamy dowolny punkt $\mathbf{x}\in \mathcal{P}$, który jest jedynym rozwiązaniem optymalnym dla pewnej funkcji celu $\mathbf{c}$.
- *Punktem ekstremalnym* wielościanu $\mathcal{P}$ nazywamy dowolny punkt $\mathbf{x}\in \mathcal{P}$, który nie jest wypukłą kombinacją dwóch innych punktów $\mathbf{y},\mathbf{z}\in\mathcal{P}$. (Przypomnijmy, że wypukła kombinacja $\mathbf{y}$ i $\mathbf{z}$ to dowolny punkt postaci $\lambda\mathbf{y}+(1-\lambda)\mathbf{z}$ dla pewnego $\lambda\in[0,1]$.)
- *Bazowe rozwiązanie dopuszczalne* (brd) PL o $n$ zmiennych to rozwiązanie dopuszczalne $\mathbf{x}\in\mathbb{R}^n$ takie, że istnieje $n$ liniowo niezależnych ograniczeń (w sensie wektorów współczynników przy $x_i$, tzn. $x_1+2x_2\ge 1$ i $x_1+2x_2\le 2$ są liniowo zależne), które dla $\mathbf{x}$ są spełnione z równością. Na przykład $(0,0)$ i $(\frac{2}{3},\frac{8}{3})$ są brd programu z [Przykładu 1](http://smurf.mimuw.edu.pl/drupal6/node/1121).

W poniższych lematach rozważamy dowolny program liniowy oznaczamy przez $\mathcal{P}$ wielościan jego rozwiązań dopuszczalnych. Zakładamy, że jest on dany w postaci $\max \mathbf{c}^T\mathbf{x}$, $\mathbf{A}\mathbf{x}\le \mathbf{b}$ (sprowadzenie dowolnego programu do takiej postaci nie zmienia wielościanu rozwiązań dopuszczalnych).

**Lemat 4**  

Jeśli $\mathbf{x}$ jest wierzchołkiem $\mathcal{P}$ to $\mathbf{x}$ jest punktem ekstremalnym.

*Dowód*  

Niech $\mathbf{c}$ będzie funkcją celu taką, że $\mathbf{x}$ jest jedynym rozwiązaniem optymalnym LP dla funkcji celu $\mathbf{c}$.

Załóżmy, że $\mathbf{x} = \lambda \mathbf{y} + (1-\lambda)\mathbf{z}$ dla pewnych $\mathbf{y},\mathbf{z}\in\mathcal{P}$ oraz $\lambda\in[0,1]$. Ponieważ $\mathbf{x}$ jest jedyny optymalny więc $\mathbf{c}^T\mathbf{y},\mathbf{c}^T\mathbf{z} < \mathbf{c}^T\mathbf{x}$.  

Ale wówczas, z liniowości $\mathbf{c}$  

$$
\mathbf{c}^T\mathbf{x}=\lambda\mathbf{c}^T\mathbf{y} + (1-\lambda)\mathbf{c}^T\mathbf{z} < \lambda\mathbf{c}^T\mathbf{x} + (1-\lambda)\mathbf{c}^T\mathbf{x} = \mathbf{c}^T\mathbf{x}.
$$  

♦

**Lemat 5**  

Jeśli $\mathbf{x}$ jest punktem ekstremalnym $\mathcal{P}$ to $\mathbf{x}$ jest bazowym rozwiązaniem dopuszczalnym.

*Dowód*  

Z definicji punktu ekstremalnego $\mathbf{x}$ jest dopuszczalny. Załóżmy, że nie jest brd, tzn. że nie istnieje $n$ liniowo niezależnych ograniczeń spełnionych z równością dla $\mathbf{x}$. Intuicyjnie, oznacza to, że wokół $\mathbf{x}$ jest nieco ,,luzu'', tzn. jest pewien wektor $\mathbf{d}$ (liniowo niezależny od wektorów ograniczeń spełnionych z równością), wzdłuż którego możemy się przemieszczać z $\mathbf{x}$ w przód i w tył pozostając w $\mathcal{P}$. Istotnie, pokażemy, że $\mathbf{x}$ jest kombinacją wypukłą dwóch punktów w $\mathcal{P}$.

Niech $T=\{i\ \vert{}\ \mathbf{a}_i^T\mathbf{x}=b_i\}$. Wiemy, że $\{\mathbf{a}_i\ \vert{}\ i\in T\}$ nie rozpina $\mathbb{R}^n$, tzn. $\mathrm{rank}[a_i, i\in T]0$, $x\pm \epsilon \mathbf{d}\in \mathcal{P}$, czyli będzie sprzeczność (bo $\mathbf{x}$ jest ekstremalny). Istotnie, jeśli $i\in T$ to dla dowolnego $\epsilon$, $\mathbf{a}_i^T(\mathbf{x}\pm \epsilon \mathbf{d})=b_i$. Dla $i\not \in T$ mamy $\mathbf{a}_i^T\mathbf{x}>b_i$, a więc na pewno można wybrać dostatecznie małe $\epsilon$, żeby $\mathbf{a}_i^T(\mathbf{x}\pm\epsilon\mathbf{d})\ge b_i$ dla każdego $i\not \in T$.  ♦

**Lemat 6**  

Jeśli $\mathbf{x}$ jest bazowym rozwiązaniem dopuszczalnym PL to $\mathbf{x}$ jest wierzchołkiem $\mathcal{P}$.

*Dowód*  

Niech $T=\{i\ \vert{}\ \mathbf{a}_i^T\mathbf{x}=b_i\}$. Podamy funkcję celu $\mathbf{c}$, przy której $\mathbf{x}$ jest jedynym rozwiązaniem optymalnym: $\mathbf{c}=\sum_{i\in T}\mathbf{a}_i$. Dla dowolnego $\mathbf{y}\in\mathcal{P}$ mamy  

$$
\mathbf{c}^T\mathbf{y}=\sum_{i\in T}\mathbf{a}_i^T\mathbf{y} \le \sum_{i\in T}b_i = \mathbf{c}^T\mathbf{x},
$$  

(czyli $\mathbf{x}$ jest rozwiązaniem optymalnym) przy czym równość zachodzi tylko gdy dla każdego $i\in T$, $\mathbf{a}_i^T\mathbf{y}=b_i$, a to jest układ którego jedynym rozwiązaniem jest $x$ (bo $\mathrm{rank}[\mathbf{a}_i\ \vert{}\ i\in T] = n$). ♦

**Wniosek 7**  

Dla dowolnego programu liniowego są równoważne:

- $\mathbf{x}$ jest wierzchołkiem,
- $\mathbf{x}$ jest punktem ekstremalnym,
- $\mathbf{x}$ jest bazowym rozwiązaniem dopuszczalnym.

**Twierdzenie 8**  

Każdy ograniczony PL w postaci standardowej $\max \mathbf{c}^T\mathbf{x}$, $\mathbf{A}\mathbf{x}\le\mathbf{b}$, $\mathbf{x}\ge\mathbf{0}$ ma rozwiązanie optymalne, które jest punktem ekstremalnym.

*Dowód*  

Niech $\mathbf{x}$ będzie rozwiązaniem optymalnym PL. Jeśli $\mathbf{x}$ jest ekstremalny -- koniec. W przeciwnym przypadku pokażemy jak przejść od $\mathbf{x}$ do punktu ekstremalnego o tej samej wartości funkcji celu (używając analogii trójwymiarowej, przesuniemy się z $\mathbf{x}$ wewnątrz wielościanu do ściany wielościanu, następnie ze środka ściany do krawędzi, a z krawędzi do wierzchołka).

Oznaczmy przez $\mathcal{P}$ wielościan rozwiązań dopuszczalnych. Skoro $\mathbf{x}$ nie jest punktem ekstremalnym, to istnieje $\mathbf{y}\ne \mathbf{0}$ taki że $\mathbf{x}+\mathbf{y},\mathbf{x}-\mathbf{y}\in\mathcal{P}$.

Po pierwsze zauważmy, że przesuwając się wzdłuż wektora $\mathbf{y}$ nie zmieniamy wartości funkcji celu. Istotnie, gdyby $\mathbf{c}^T \mathbf{y} > \mathbf{0}$ to $\mathbf{c}^T (\mathbf{x}+\mathbf{y}) > \mathbf{c}^T \mathbf{x}$, natomiast gdyby $\mathbf{c}^T \mathbf{y} < \mathbf{0}$ to $\mathbf{c}^T (\mathbf{x}-\mathbf{y}) > \mathbf{c}^T \mathbf{x}$, czyli w obu przypadkach dostajemy sprzeczność z założeniem że $\mathbf{x}$ jest ekstremalny (a więc jest wierzchołkiem). Skoro $\mathbf{c}^T\mathbf{y} = \mathbf{0}$ to dla dowolnego $\alpha$, $\mathbf{c}^T(\mathbf{x}+\alpha\mathbf{y})=\mathbf{c}^T\mathbf{x}$.

Rozważmy teraz dowolne ograniczenie spełnione z równością, czyli $(\mathbf{a}_i)^T\mathbf{x} = \mathbf{b}_i$. Teraz zauważmy, że skoro $(\mathbf{a}_i)^T (\mathbf{x}+\mathbf{y}) \le \mathbf{b}$ to $(\mathbf{a}_i)^T \mathbf{y} \le 0$. Podobnie skoro $(\mathbf{a}_i)^T (\mathbf{x}-\mathbf{y}) \le \mathbf{b}$ to $(\mathbf{a}_i)^T \mathbf{y} \ge 0$. Stąd $(\mathbf{a}_i)^T\mathbf{y} = \mathbf{0}$ a nawet $(\mathbf{a}_i)^T(\alpha\mathbf{y}) = \mathbf{0}$ dla dowolnego $\alpha\in\mathbb{R}$. Czyli jeśli $i$-te ograniczenie jest spełnione z równością, to po przesunięciu się wzdłuż $\mathbf{y}$ dalej tak będzie, tzn.\ $(\mathbf{a}_i)^T(\mathbf{x}+\alpha \mathbf{y}) = \mathbf{b}_i$.  

Geometrycznie, odpowiada to spostrzeżeniu, że jeśli $\mathbf{x}$ jest na krawędzi (ścianie, ...), to posuwając się wzdłuż $\mathbf{y}$ dalej pozostajemy na krawędzi (ścianie, ...).

Niech teraz $\lambda = \max\{\alpha\ \vert{}\ \mathbf{x} + \alpha \mathbf{y} \in \mathcal{P} \text{ oraz } \mathbf{x} - \alpha \mathbf{y} \in \mathcal{P} \}$. Weźmy $j$ takie, że $y_j \ne 0$. Zauważmy, że jeśli $x_j = 0$ to $y_j = 0$ (wpp. $x_j+y_j<0$ lub $x_j-y_j<0$, czyli $\mathbf{x}+\mathbf{y}\not\in\mathcal{P}$ lub $\mathbf{x}-\mathbf{y}\not\in\mathcal{P}$), a więc musi być $x_j \ne 0$. Oznacza to, że dla dostatecznie dużego $\alpha$, $x_j + \alpha y_j = 0$ lub $x_j - \alpha y_j = 0$, czyli $\lambda$ jest dobrze określone. Zauważmy, że w rozwiązaniu dopuszczalnym $\mathbf{x}+\lambda \mathbf{y}$ *rośnie* liczba ograniczeń spełnionych z równością. To implikuje, że powyższą operację wykonamy co najwyżej $n+m$ razy zanim dojdziemy do punktu ekstremalnego. ♦

**Wniosek 9**  

Istnieje algorytm (siłowy), który rozwiązuje PL w postaci standardowej o $n$ zmiennych i $m$ ograniczeniach w czasie $O({m\choose n}n^3)$.

*Dowód*  

Żeby rozwiązać LP w postaci standardowej wystarczy sprawdzić wszystkie wierzchołki wielościanu i wybrać wierzchołek o najmniejszej wartości funkcji celu.  Wierzchołków jest tyle co bazowych rozwiązań dopuszczalnych, czyli $m\choose n$. Aby znaleźć taki wierzchołek wystarczy rozwiązać układ odpowiednich $n$ liniowo niezależnych równań. ♦

(Uwaga. W powyższym twierdzeniu $m$ oznacza liczbę ograniczeń, a *nie* liczbę wierszy macierzy $\mathbf{A}$. Jeśli tę liczbę wierszy oznaczymy przez $r$ to $m=r+n$).

Podsumowując, nasze rozważania geometryczne pozwoliły nam zmienić ,,naturę problemu'': w pewnym sensie przeszliśmy od problemu o charakterze ciągłym (continuum rozwiązań dopuszczalnych) do problemu dyskretnego.
