---
layout: 'default'
title: 'Algorytmy aproksymacyjne 2: algorytmy oparte na programowaniu liniowym'
page_url: 'http://smurf.mimuw.edu.pl/node/1147'
source_url: 'http://smurf.mimuw.edu.pl/node/1147'
source_kind: 'html'
uses_math: true
render_with_liquid: false
rendered_file: 'lectures/zaawansowane-algorytmy-i-struktury-danych/algorytmy-aproksymacyjne-2-algorytmy-oparte-na-programowaniu-liniowym/rendered.html'
---

# Algorytmy aproksymacyjne 2: algorytmy oparte na programowaniu liniowym

### Program liniowy jako ,,pośrednik'' między algorytmem a rozwiązaniem optymalnym

W poprzednim wykładzie poznaliśmy algorytm 2-aproksymacyjny dla problemu pokrycia wierzchołkowego. Bardzo naturalna jest ważona wersja tego problemu, w której dodatkowo dana jest funkcja $w:V\rightarrow \mathbb{R}$, tzn. każdy wierzchołek $v$ ma swoją *wagę* $w(v)$. Celem jest oczywiście znalezienie pokrycia wierzchołkowego o najmniejszej możliwej wadze (tzn. sumie wag wierzchołków pokrycia). Czy dla tak uogólnionego problemu również jesteśmy w stanie podać algorytm aproksymacyjny? Pamiętamy, że kluczowym krokiem podczas budowania takiego algorytmu było znalezienie tzw. *pośrednika*, tzn. obiektu, który możemy znaleźć w czasie wielomianowym, oraz takiego, że jego koszt (waga) jest mniejszy niż koszt rozwiązania optymalnego, a równocześnie możemy na jego podstawie uzyskać rozwiązanie dopuszczalne, przy pewnej stracie na koszcie. W prostszej wersji problemu pośrednikiem było maksymalne skojarzenie. Jakiego pośrednika użyjemy tym razem? W algorytmie dla metrycznego problemu komiwojażera wspomnieliśmy, że często pośrednika uzyskujemy rozluźniając wymagania dotyczące poszukiwanego obiektu. Aby pójść tym tropem wystarczy już tylko skojarzyć dwa fakty: wiele problemów optymalizacyjnych możemy wyrazić jako [całkowitoliczbowe programy liniowe](http://smurf.mimuw.edu.pl/drupal6/node/1126), natomiast po usunięciu warunku całkowitoliczbowości otrzymujemy program liniowy (zwany relaksacją), który możemy rozwiązać w czasie wielomianowym. Oto całkowitoliczbowy program liniowy dla problemu ważonego pokrycia wierzchołkowego:

Program (IWVC)  

$$
\label{eq:vc-3}  

       \begin{array}{rll}  

       \textrm{min}                    & \sum_{v\in V} w(v)x_v \\  

                                       & x_u+x_v \ge 1 & \forall uv \in E\\  

                                       & x_v \in \{0,1\} & \forall v \in V.  

       \end{array}
$$

Po opuszczeniu warunku całkowitoliczbowości otrzymujemy program liniowy:

Program (WVC0)  

$$
\begin{array}{rll}  

       \textrm{min}                    & \sum_{v\in V} w(v)x_v \\  

                                       & x_u+x_v \ge 1 & \forall uv \in E\\  

                                       & x_v \ge 0 & \forall v \in V.\\  

                                       & x_v \le 1 & \forall v \in V.  

       \end{array}
$$

Możemy zauważyć, że warunek $x_v \le 1$ jest tu zbędny ponieważ nawet jeśli zostanie usunięty, to w każdym rozwiązaniu *optymalnym* i tak będzie on spełniony. Otrzymujemy więc następujący program:

Program (WVC)  

$$
\begin{array}{rll}  

       \textrm{min}                    & \sum_{v\in V} w(v)x_v \\  

                                       & x_u+x_v \ge 1 & \forall uv \in E\\  

                                       & x_v \ge 0 & \forall v \in V.  

       \end{array}
$$

Przez ${\rm OPT}$ będziemy oznaczać zawsze rozwiązanie optymalne oryginalnego problemu (programu całkowitoliczbowego), natomiast przez ${\rm OPT}_f$ tzw. *rozwiązanie ułamkowe*, czyli rozwiązanie optymalne relaksacji programu całkowitoliczbowego. O ile nie będzie to prowadzić do niejednoznaczności, tymi samymi symbolami będziemy oznaczać koszty tych rozwiązań. W przypadku problemów minimalizacji prawdziwa jest nierówność  

$$
{\rm OPT}_f \le {\rm OPT}.
$$

Widzimy więc, że ,,pośrednikiem'' może być rozwiązanie optymalne programu (WVC).

### Zaokrąglanie w problemie ważonego pokrycia wierzchołkowego

Rozwiązanie ${\rm OPT}_f$ możemy znaleźć w czasie wielomianowym algorytmem Khachiyana lub Karmarkara. Pozostaje jedynie przekształcić je do rozwiązania dopuszczalnego oryginalnego problemu. Oznaczmy wartości zmiennych ${\rm OPT}_f$ przez $x^*_v$ dla $v\in V$. Spróbujmy zastosować pierwszy pomysł jaki się tu narzuca: zaokrąglimy wartości zmiennych $x^*_v$. Przyjmijmy, że $x_v = 1$ gdy $x^*_v \ge \frac{1}{2}$ oraz $x_v = 0$ w przeciwnym przypadku. Nasz algorytm zwraca zbiór $A = \{v \in V\ : \ x_v = 1\}$.

**Lemat** $A$ jest pokryciem wierzchołkowym.  

*Dowód* Rozważmy dowolną krawędź $uv \in E$ i załóżmy, że nie jest ona pokryta, tzn. $\{u,v\} \cap A = \emptyset$. Wówczas $x^*_u < \frac{1}{2}$ oraz $x^*_v < \frac{1}{2}$, co implikuje $x^*_u + x^*_v < 1$, a więc rozwiązanie ${\rm OPT}_f$ nie jest dopuszczalne, sprzeczność ♦

Pozostaje jedynie oszacować jak bardzo zwiększyła się funkcja celu podczas operacji zaokrąglania zmiennych. Zauważmy, że dla dowolnego wierzchołka $v$, $x_v \le 2 x^*_v$. Stąd,

$$
w(A) = \sum_{v\in V} w(v)x_v \le \sum_{v\in V} 2 w(v)x^*_v = 2 {\rm OPT}_f  \le 2 {\rm OPT}.
$$

**Wniosek** Istnieje algorytm 2-aproksymacyjny dla problemu ważonego pokrycia wierzchołkowego.

### Zaokrąglanie w problemie ważonego pokrycia zbioru

Efekt, jaki uzyskaliśmy jest dość zdumiewający: korzystając z programowania liniowego, niemal automatycznie dostaliśmy algorytm 2-aproksymacyjny dla nowej, trudniejszej wersji problemu. Co więcej, wydaje się, że użyta metoda jest dość uniwersalna i może mieć zastosowanie w wielu innych problemach. Dla przykładu, rozważmy problem pokrycia zbioru:

Problem pokrycia zbioru  

**Dane:** Zbiory $S_1, \ldots, S_m$ oraz ich koszty $w:\{1,\ldots,m\} \rightarrow \mathbb{R}$.  

**Problem:** Niech $U=\bigcup_{i=1}^m S_i$. Należy pokryć $U$ zbiorami $S_i$ w najtańszy możliwy sposób, tzn. znaleźć zbiór indeksów $I \subseteq \{1,\ldots,m\}$ taki, że $U=\bigcup_{i\in I} S_i$ oraz $\sum_{i\in I}w(i)$ jest najmniejsze możliwe.

Powyższy problem jest oczywiście NP-trudny, gdyż pokrycie wierzchołkowe jest jego szczególnym przypadkiem: wystarczy przyjąć $U = E$ oraz dla każdego wierzchołka $v \in V$ określić zbiór $S_v$ złożony ze wszystkich krawędzi incydentnych z $v$. Problem pokrycia zbioru również możemy wyrazić jako całkowitoliczbowy program liniowy:

Program (IWSC)  

$$
\begin{array}{rll}  

       \textrm{min}                    & \sum_{i=1}^m w(i)x_i \\  

                                       & \sum_{i\ :\ S_i \ni e} x_i \ge 1 & \forall e \in U\\  

                                       & x_i \in \{0,1\} & \forall i=1,\ldots,m.  

       \end{array}
$$

Jego relaksacja wygląda następująco:

Program (WSC)  

$$
\begin{array}{rll}  

       \textrm{min}                    & \sum_{i=1}^m w(i)x_i \\  

                                       & \sum_{i\ :\ S_i \ni e} x_i \ge 1 & \forall e \in U\\  

                                       & x_i \ge 0 & \forall i=1,\ldots,m.  

       \end{array}
$$

Spróbujmy postępować analogicznie jak poprzednio: nasz algorytm zaczyna od znalezienia rozwiązania optymalnego ${\rm OPT}_f=(x^*_1,\ldots,x^*_m)$ programu (WSC). Tym razem proste zaokrąglenie do najbliższej liczby całkowitej nie wystarcza aby otrzymać pokrycie, czyli rozwiązanie dopuszczalne programu (IWSC): jest tak dlatego, że dla elementu $u \in U$ może się okazać, że dla każdego zbioru $S_i$ zawierającego $u$ mamy $x^*_i < \frac{1}{2}$. Pokrycie wierzchołkowe to szczególny przypadek pokrycia zbioru, w którym *każdy element należy do dokładnie 2 zbiorów*. Wówczas, z lewej strony nierówności $\sum_{i\ :\ S_i \ni e} x^*_i \ge 1$ mamy tylko 2 zmienne i jedna z nich musi być równa co najmniej $\frac{1}{2}$. Analogicznego argumentu moglibyśmy użyć w problemie pokrycia zbioru, gdyby liczba zbiorów zawierających dany element była ograniczona przez jakąś niewielką stałą, powiedzmy stałą $f$. Wówczas dla dowolnego $u\in U$ mamy pewność, że dla pewnego zbioru $S_i$ zawierającego $u$ mamy $x^*_i \ge \frac{1}{f}$. Zaokrąglając takie zmienne do 1, a pozostałe do 0, otrzymamy rozwiązanie dopuszczalne.

Algorytm $f$-aproksymacyjny dla problemu ważonego pokrycia zbioru  

1. Znajdź rozwiązanie optymalne $\mathbb{x}^* = (x^*_1,\ldots,x^*_m)$ programu (WSC).
2. Dla każdego $i=1,\ldots,m$ przyjmij: $$
x_i = \left\{ \begin{array}{cl} 1 & \text{gdy $x^*_i \ge \frac{1}{f}$} \\ 0 & \text{w p.p.} \end{array} \right.
$$
3. Zwróć pokrycie $I = \{i \ :\ x_i = 1\}$.

Analogicznie jak poprzednio, $x_i \le f x^*_i$, więc szacujemy

$$
\sum_{i\in I}w(i) = \sum_{i=1}^m w(i)x_i \le \sum_{i=1}^m f w(i)x^*_i = f {\rm OPT}_f \le f {\rm OPT}.
$$

**Wniosek**  

Jeśli każdy element zbioru $U$ występuje w co najwyżej $f$ zbiorach $S_i$, to powyższy algorytm jest $f$-aproksymacyjny.

Czy tworząc jakąś bardzo pomysłową regułę zaokrąglania lub może za pomocą zupełnie innego podejścia możemy uzyskać algorytm aproksymacyjny ze współczynnikiem aproksymacji ograniczonym przez stałą, niezależnym od $f$? Niestety, okazuje się, że jest to najprawdopodobniej niemożliwe, zachodzi bowiem następujące twierdzenie, którego dowód znacznie wykracza poza zakres naszego wykładu:

**Twierdzenie (Raz, Safra 1997)** Nie istnieje algorytm $o(\log\vert{}U\vert{})$-aproksymacyjny dla problemu pokrycia zbioru, o ile ${\rm P} \ne {\rm NP}$.

W tym wykładzie zobaczyliśmy, że programy liniowe -- relaksacje odpowiednich programów całkowitoliczbowych, mogą być punktem wyjścia do rozważań o algorytmach aproksymacyjnych. Istotnie, obecnie dla większości problemów najlepsze algorytmy uzyskuje się stosując metody oparte na programowaniu liniowym. W tym wykładzie poznaliśmy przykład zastosowania jednej z takich metod: zaokrąglanie, tzn. zamiana rozwiązania ułamkowego na całkowitoliczbowe. Konkretny algorytm zaokrąglania zależy od problemu: bardzo rzadko jest to zwykłe zaokrąglenie do najbliższej liczby całkowitej.

### Metoda prymalno-dualna

Z pomocą programowania liniowego udało nam się pokazać, że problem pokrycia wierzchołkowego ma 2-aproksymację nawet w wersji ważonej. Niemniej, z tym uogólnieniem związane są pewne koszty: w wersji nieważonej nasz algorytm wymagał jedynie znalezienia maksymalnego ze względu na zawieranie skojarzenia, co można trywialnie zrealizować w czasie liniowym, tymczasem metoda zaokrąglania w wersji ważonej wymaga użycia skomplikowanego algorytmu do rozwiązywania programów liniowych, którego czas działania wyraża się wielomianem dość dużego stopnia. Pojawia się pytanie, czy w wersji ważonej możliwe jest uzyskanie 2-aproksymacji za pomocą prostszego (a może przy okazji również szybszego) algorytmu. Okazuje się, że jest to możliwe, niemniej jednak teoria programowania liniowego dalej będzie odgrywać kluczową rolę. Pomocny okaże się program dualny do programu (WVC):

Program (D-WVC)  

$$
\begin{array}{rll}  

       \textrm{max}                    & \sum_{e \in E} y_e \\  

                                       & \sum_{vw\in E}y_{vw} \le w(v) & \forall v \in V\\  

                                       & y_e \ge 0 & \forall e \in E.\\  

       \end{array}
$$

Zaczniemy od następującej obserwacji.

**Obserwacja 1**  

Niech $y$ będzie dowolnym rozwiązaniem dopuszczalnym programu (D-WVC). Wówczas, dla dowolnej krawędzi $e = uv \in E$, zachodzi co najmniej jeden z poniższych warunków:

1. $\sum_{vw\in E}y_{vw} = w(v)$,
2. $\sum_{uw\in E}y_{uw} = w(u)$,
3. po powiększeniu $y_e$ o $\min\{w(v) - \sum_{vw\in E}y_{vw}, w(u) - \sum_{uw\in E}y_{uw}\}$ otrzymamy nowe rozwiązanie dopuszczalne.

Obserwacja 1 sugeruje następujący prosty algorytm:

Algorytm prymalno-dualny  

1. $\mathbf{y} := \mathbf{0}$ (początkowe, zerowe rozwiązanie dopuszczalne programu (D-WVC))
2. Dopóki istnieje krawędź $e$ taka, że żaden z warunków 1 i 2 nie jest spełniony (tzn. $\sum_{vw\in E}y_{vw} < w(v)$ oraz $\sum_{uw\in E}y_{uw} < w(u)$), zmodyfikuj $\mathbf{y}$, zgodnie z warunkiem 3.
3. Zwróć zbiór $S = \{v \in V\ :\ \sum_{vw \in E}y_{vw} = w(v)\}$.

Z obserwacji 1 jest oczywiste, że algorytm zwraca pokrycie. Ponieważ przy każdym obrocie pętli co najmniej 1 nowa nierówność staje się równością (a te nierówności, które już były spełnione z równością dalej takimi pozostają), wykona się co najwyżej $\vert{}V\vert{}$ obrotów pętli. Cały algorytm można łatwo zaimplementować w czasie $O(\vert{}V\vert{}+\vert{}E\vert{})$.

Nazwa ,,algorytm prymalno-dualny'' bierze się stąd, że możemy przyjąć, że w każdym momencie działania algorytmu, oprócz rozwiązania $\mathbf{y}$ programu dualnego określone jest również rozwiązanie $\mathbf{x}$ programu prymalnego (używamy notacji Iversona):  

$$
x_v = [\sum_{vw \in E}y_{vw} = w(v)].
$$  

Algorytm utrzymuje więc dopuszczalne rozwiązanie programu dualnego i całkowitoliczbowe, ale niekoniecznie dopuszczalne, rozwiązanie programu prymalnego. Niespełniony warunek programu prymalnego podpowiada, którą zmienną dualną należy zmienić. Z kolei warunek programu dualnego, który staje się spełniony z równością określa zmienną prymalną, która zmienia swoją wartość. W ten sposób algorytm na zmianę znajduje rozwiązania programu dualnego i prymalnego, jedno ,,coraz bardziej optymalne'', drugie ,,coraz bardziej dopuszczalne''. W chwili gdy rozwiązanie prymalne staje się dopuszczalne, algorytm się kończy.

Pozostaje oszacować wagę uzyskanego pokrycia wierzchołkowego:

$$
w(S)=\sum_{v \in S} w(v) = \sum_{v \in S} \sum_{vw\in E}y_{vw} \le 2 \sum_{e \in E} y_{e} \le 2 {\rm OPT}.
$$

Pierwsza nierówność bierze się tu stąd, że ponieważ krawędź ma tylko 2 końce, więc dla każdej krawędzi $e$ zmienna $y_e$ pojawi się w podwójnej sumie co najwyżej 2 razy. Druga nierówność bierze się natomiast ze słabej dualności: $\sum_{e \in E} y_{e}$ jest funkcją celu dla rozwiązania dopuszczalnego $\mathbf{y}$ programu dualnego, a optymalne pokrycie, czyli rozwiązanie optymalne programu (IWVC) jest rozwiązaniem dopuszczalnym programu prymalnego (WVC).

Na powyższe szacowanie warto patrzeć w następujący, intuicyjny sposób. Pokrycie $S$ jest opłacone przez sumę wszystkich zmiennych dualnych znajdujących się w warunkach spełnionych z równością. Każda zmienna dualna pojawia się w dwóch warunkach, a więc płaci co najwyżej dwa razy. Stąd, całkowity koszt nie przekracza $2 \sum_{e\in E} y_e$. Taki ,,mechanizm lokalnych opłat'' pojawia się w wielu algorytmach aproksymacyjnych.

**Wniosek** Algorytm prymalno-dualny jest 2-aproksymacyjny.

Warto zauważyć, że w algorytmie prymalno-dualnym użyliśmy nowego ,,pośrednika'': było to rozwiązanie dopuszczalne programu dualnego. Dzięki temu, że nie potrzebowaliśmy rozwiązania optymalnego programu dualnego, nasz algorytm jest bardzo prosty w implementacji.

A teraz zagadka: Jaki algorytm otrzymamy, jeśli zastosujemy algorytm prymalno-dualny w sytuacji, gdy wszystkie wagi wierzchołków są równe 1 (czyli de facto w nieważonej wersji problemu pokrycia wierzchołkowego)?

Rozwiązanie zagadki  

Łatwo zauważyć, że podczas działania algorytmu każda zmiana zmiennej dualnej będzie polegać na zwiększeniu jej z 0 do 1. Warunki programu dualnego implikują, że w każdej chwili działania algorytmu zbiór $M=\{e\ : \ y_e = 1\}$ jest skojarzeniem w wejściowym grafie. Co więcej, jeśli $v$ jest końcem krawędzi $e \in M$, to $\sum_{vw\in E}y_{vw} = 1 = w(v)$, czyli $v \in S$, a więc $S = V(M)$. Algorytm kończy się, gdy $M$ jest maksymalne ze względu na zawieranie. Podsumowując, jest to dokładnie prosty algorytm z [wykładu o algorytmach kombinatorycznych](http://smurf.mimuw.edu.pl/drupal6/node/1142).

Metoda prymalno-dualna została z sukcesem zastosowana w wielu problemach. Historycznie pierwszym takim problemem był problem najtańszego skojarzenia w grafie dwudzielnym (był to algorytm dokładny, a nie aproksymacyjny). Zachęcamy czytelnika do rozwiązania poniższego zadania.

**Zadanie** Opisz algorytm prymalno-dualny dla problemu ważonego pokrycia zbioru i udowodnij, że jest on $f$-aproksymacyjny.
