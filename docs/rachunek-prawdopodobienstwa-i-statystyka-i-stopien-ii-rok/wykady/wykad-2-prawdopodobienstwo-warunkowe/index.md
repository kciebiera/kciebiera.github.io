---
title: "Wykład 2: Prawdopodobieństwo warunkowe"
source_url: "http://smurf.mimuw.edu.pl/node/707"
source_kind: "html"
---

# Wykład 2: Prawdopodobieństwo warunkowe

## Motywujący przykład

**Przykład 2.1**  

Test na obecność pewnego wirusa ma skuteczność 95%, tzn. jeśli badana osoba jest chora (t.j. zakażona wirusem), to z prawdopodobieństwem 0.95 test daje wynik pozytywny, a jeśli badana osoba jest zdrowa, to z prawdopodobieństwem 0.95 test daje wynik negatywny. Wiadomo, że średnio 1 osoba na 1000 jest zakażona. Jeśli dla osoby wybranej losowo test dał wynik pozytywny, to jakie jest prawdopodobieństwo tego, że osoba ta jest faktycznie chora?

Powyższe pytanie jest o tyle ciekawe, że choć większość osób ma swoją "teorię" na temat szukanego prawdopodobieństwa, to z reguły jest ona niestety błędna. W amerykańskich szpitalach przeprowadzono kiedyś wśród lekarzy ankietę dotyczącą tego właśnie problemu i wyniki były zatrważające. Większość ankietowanych podało odpowiedź różniącą się rzędem wielkości od prawidłowej!

Próbując rozwiązać powyższy problem zdefiniujemy pojęcie prawdopodobieństwa warunkowego i wyprowadzimy wiele jego własności. Pojęcie to, jak się okaże, ma wiele zastosowań, także w sytuacjach pozornie nie mających z nim nic wspólnego.

---

## Definicja prawdopodobieństwa warunkowego

Zanim odpowiemy na pytanie postawione w przykładzie, musimy się zastanowić nad tym jaki jest jego sens. Czym jest "prawdopodobieństwo $A$ jeśli wiemy, że zaszło $B$" ?. Przyjrzyjmy się najpierw dużo prostszemu przykładowi.  

**Przykład 2.2**  

Nasz kolega rzuca dwiema kostkami i podaje nam sumę oczek. Załóżmy, że podaną sumą jest 5. Jakie jest prawdopodobieństwo, że na pierwszej kostce wypadły 1 lub 2 oczka?

Informacja podana przez kolegę ograniczyła zbiór zdarzeń elementarnych do $A=\{(1,4),(2,3),(3,2)(4,1)\}$. Zbiór $A$ zawiera 4 elementy, ale tylko dwa z nich odpowiadają wypadnięciu 1 lub 2 oczek na pierwszej kostce. Sądzimy więc, że szukane prawdopodobieństwo wynosi $\frac{1}{2}$.

Co dokładnie się tu stało? Obliczyliśmy wartość wyrażenia $\frac{|A\cap B|}{|A|}=\frac{P(A \cap B)}{P(A)}$, gdzie $A$ jest zdarzeniem, o którym wiemy, że zaszło, a $B$ zdarzeniem, którego prawdopodobieństwo chcemy obliczyć. Pierwsze z wyrażeń ma tę wadę, że implicite zakładamy w nim użycie schematu klasycznego. Drugie wyrażenie wady tej nie ma i to jego właśnie użyjemy w definicji:

**Definicja (Prawdopodobieństwo warunkowe)**  

Niech $A,B \subseteq \Omega$ i $P(A) > 0$. Wtedy *prawdopodobieństwem $B$ pod warunkiem $A$* nazywamy:  

$P(B|A) = \frac{P(A \cap B)}{P(A)}$.

**Uwaga 2.3**  

Założenie $P(A) > 0$ jest konieczne, aby wartość ilorazu była dobrze określona. Można sobie jednak wyobrazić sytuacje, w których $P(A)=0$, a mimo to pojęcie prawdopodobieństwa warunkowego wydaje się mieć sens. Jeśli na przykład wykonujemy nieskończenie wiele rzutów kostką, to prawdopodobieństwo wyrzucenia za każdym razem tej samej liczby oczek jest równe 0. Jeśli jednak się to zdarzy, to prawdopodobieństwo tego, że były to jedynki powinno być równe $\frac{1}{6}$. W ogólnym przypadku nie da się w sensowny sposób zdefiniować prawdopodobieństwa warunkowego tak, aby obejmowało ono tego typu sytuacje. Sprawa nie jest jednak całkiem beznadziejna i pewne próby takiej definicji zobaczymy w rozdziale poświęconym zmiennym losowym o rozkładach ciągłych.

**Uwaga 2.4**  

Warto zauważyć, że para $(\Omega, P(\cdot|A))$ jest przestrzenią probabilistyczną (czytelnikowi pozostawiamy sprawdzenie aksjomatów). Co za tym idzie, wiele z przeprowadzanych przez nas rozważań ma swoje odpowiedniki "warunkowe".

---

## Prawdopodobieństwo iloczynu zdarzeń

**Przykład 2.1 (c.d.)**  

Wróćmy do przykładu z testem medycznym. Zdefiniujmy następujące zdarzenia:

- $C$ - wybrana osoba jest chora,
- $Z$ - wybrana osoba jest zdrowa,
- $T$ - test dał wynik pozytywny,
- $N$ - test dał wynik negatywny,

Uzbrojeni w definicję prawdopodobieństwa warunkowego łatwo zauważymy dwie rzeczy. Po pierwsze, naszym celem jest obliczenie $P(C|T)$. Po drugie, dane z treści zadania opisują pewne prawdopodobieństwa warunkowe:

- $P(T|C) = 0.95$,
- $P(N|C) = 0.05$,
- $P(T|Z) = 0.05$,
- $P(N|Z) = 0.95$.

Aby znaleźć interesującą nas wartość  

$P(C|T) = \frac{P(C \cap T)}{P(T)}$  

spróbujmy najpierw obliczyć $P(C \cap T)$? Popatrzmy na definicję prawdopodobieństwa warunkowego:  

$P(T|C) = \frac{P(T \cap C)}{P(C)}$.  

Widać, że:  

$P(T \cap C) = P(C) P(T|C)$.

Sytuacja, z którą mamy tu do czynienia, t.j. obliczanie prawdopodobieństwa iloczynu zdarzeń za pomocą prawdopodobieństwa warunkowego, jest bardzo częsta i ważna. W szczególności istnieje uogólnienie wzoru, który uzyskaliśmy powyżej na więcej niż dwa zdarzenia.

**Twierdzenie 2.5 (Wzór iloczynowy)**  

Niech $A_1,\ldots,A_n \subseteq \Omega$ będą zdarzeniami takimi, że $P(A_1 \cap \ldots \cap A_{n-1}) > 0$. Wtedy  

$P(A_1 \cap \ldots \cap A_n) = P(A_1) \cdot P(A_2 | A_1) \cdot P(A_3 | A_1 \cap A_2) \cdot \ldots \cdot P(A_n | A_1 \cap \ldots \cap A_{n-1})$

**Uwaga 2.6**  

Założenie $P(A_1 \cap \ldots \cap A_{n-1}) > 0$ jest potrzebne, żeby wszystkie prawdopodobieństwa warunkowe występujące w tezie twierdzenia były dobrze określone.

**Dowód**  

Indukcja. Dla dwóch zdarzeń twierdzenie pokazaliśmy już wcześniej. Jeśli mamy $n > 2$ zdarzenia, to  

$P(A_1 \cap \ldots \cap A_n) = P((A_1 \cap \ldots \cap A_{n-1}) \cap A_n) = P(A_1 \cap \ldots \cap A_{n-1}) \cdot P(A_n | A_1 \cap \ldots \cap A_{n-1})$  

z tezy dla dwóch zdarzeń. Wystarczy teraz zastosować do pierwszego członu założenie indukcyjne.

---

## Wzór na prawdopodobieństwo całkowite i twierdzenie Bayesa

**Przykład 2.1 (c.d.)**  

Wróćmy do przykładu z testem medycznym. Potrafimy obliczyć prawdopodobieństwa $P(C \cap T)$, i w analogiczny sposób także $P(C\cap N), P(Z \cap T)$ i $P(Z\cap N)$.  

Aby obliczyć szukane przez nas $P(C|T) = \frac{P(C \cap T)}{P(T)}$, potrzebujemy jeszcze znaleźć wartość $P(T)$. Ale to jest proste:  

$P(T) = P(C\cap T+Z\cap T) = P(C\cap T)+P(Z\cap T) = P(C)  P(T|C) + P(Z) P(T|Z)$.  

Ostatecznie dostajemy więc:  

$P(C|T) = \frac{P(C)  P(T|C)}{P(C)  P(T|C) + P(Z) P(T|Z)} = \frac{0.001 \cdot 0.95}{0.001 \cdot 0.95 + 0.999 \cdot 0.05} \sim \frac{0.001}{0.05} = 0.02.$  

**Komentarz**  

Jeśli sądziłeś, tak jak większość ankietowanych lekarzy, że $P(C|T)$ jest bliskie $0.95$, to spróbuj w tej chwili znaleźć prosty powód dla którego wynik ten nie jest możliwy.

Aby zobaczyć, że wynik ten nie jest możliwy wystarczy wykonać prosty eksperyment myślowy. Losowo wybrana osoba jest chora z prawdopodobieństwem $0.001$. Jeśli na losowo wybranej osobie przeprowadzimy test, to da on wynik pozytywny z prawdopodobieństwem co najmniej $0.05$, niezależnie od tego, czy osoba ta jest chora, czy nie. Gdyby faktycznie zachodziło $P(C|T) \sim 0.95$, to oznaczałoby to, że prawdopodobieństwo tego, że mamy do czynienia z osobą chorą w magiczny sposób rośnie tylko przez to, że ją badamy.

Należy jednak przyznać, że odpowiedzi udzielane przez lekarzy nie są aż tak bardzo odległe od prawdy jak mogłoby nam się wydawać. Lekarze nie są bowiem przyzwyczajeni do pacjentów, którzy są losowo wybierani z całej populacji. U pacjentów, którzy są poddawani testom medycznym z reguły występują objawy zgodne z diagnozowaną chorobą. W takiej sytuacji $P(C)$ jest dużo większe, niż odpowiednia wartość dla całej populacji.

Sposób w jaki obliczyliśmy $P(T)$  jest szczególnym przypadkiem bardzo przydatnego twierdzenia:  

**Twierdzenie 2.7(Wzór na prawdopodobieństwo całkowite)**  

Niech $B,A_1,A_2,\ldots \subseteq \Omega$ będą takie, że $P(A_k) > 0$ dla wszystkich $k$, i ponadto niech zbiory $A_1,A_2,\ldots$ stanowią podział $\Omega$, t.j. niech będą parami rozłączne i $A_1 \cup A_2 \cup \ldots  = \Omega$. Wtedy  

$P(B) = \sum_{k=1}^\infty P(A_k) P(B|A_k).$

**Dowód**  

$P(B) = P(B \cap (A_1 \cup A_2 \cup \ldots)) = P((B \cap A_1) \cup  P((B \cap A_2) \cup \ldots) = \sum_{k=1}^\infty P(B \cap A_k) = \sum_{k=1}^\infty P(A_k) P(B|A_k).$

O wzorze na prawdopodobieństwo całkowite można myśleć jako o metodzie obliczania prawdopodobieństwa "przez przypadki", ew. metodą "dziel i rządź". Ze wzoru tego w prosty sposób wynika tzw. wzór Bayesa, który jest uogólnieniem sposobu, w jaki obliczaliśmy $P(C|T)$.

**Twierdzenie 2.8 (Wzór Bayesa)**  

Niech $B,A_1,A_2,\ldots$ będą takie jak poprzednio. Wtedy  

$P(A_i|B) = \frac{P(A_i \cap B)}{P(B)}=\frac{P(A_i)P(B|A_i)}{\sum_{k=1}^\infty P(A_k) P(B|A_k)} .$  

**Dowód**  

Teza została sformułowana w taki sposób, że od razu jest swoim dowodem.

**Uwaga**  

Zarówno wzór na prawdopodobieństwo całkowite, jak i twierdzenie Bayesa zachodzą także dla skończonych podziałów $\Omega$, dowody są analogiczne do przedstawionych powyżej. Nie wynika to wprost z wersji dla podziałów nieskończonych ze względu na założenie $P(A_k)>0$.

Warto w tym miejscu zaznaczyć, że twierdzenie Bayesa jest fundamentalnym narzędziem probabilistycznym o licznych zastosowaniach. Duża część z nich ma następujący charakter:

1. Modelujemy naszą wiedzę na temat pewnego obiektu/procesu przypisując prawdopodobieństwo $P(S_i)$ każdemu jego stanowi/przebiegowi $S_i$.
2. Uzyskujemy nową informację $I$ na temat badanego obiektu, przy czym znamy dla każdego stanu obiektu prawdopodobieństwo $P(I|S_i)$ uzyskania tej właśnie informacji jeśli obiekt znajduje się w stanie $S_i$.
3. Korzystając ze wzoru Bayesa obliczamy nowe prawdopodobieństwa $P(S_i|I)$ uwzględniające informację I.

A oto kilka przykładowych sytuacji tego typu, z zupełnie różnych dziedzin:

- Tworzymy system ekspercki wspomagający diagnostykę medyczną. W tym przypadku zdarzenia $S_i$ odpowiadają różnym chorobom, ew. kombinacjom chorób, natomiast informacja $I$ może być wynikiem testu medycznego, nowym objawem itp.
- Tworzymy system wspomagający poszukiwania wraków statków na dnie morza. W tym przypadku zdarzenia $S_i$ mogą odpowiadać różnym obszarom poszukiwań, a informacją $I$ może być zakończony niepowodzeniem dzień poszukiwań w konkretnym obszarze. Wtedy, jeśli przeszukiwanym obszarem był obszar $k$-ty, to $P(S_k | I) < P(S_k)$ oraz $P(S_i | I) > P(S_i)$ dla $i \neq k$, a dokładne wartości $P(S_i|I)$ można obliczyć ze wzoru Bayesa.
- Tworzymy program grający z w pokera. Program ten modeluje styl gry swojego przeciwnika za pomocą stanów $S_i$, np. poszczególne stany mogą odpowiadać różnym stopniom agresji. Jeśli przeciwnik jest pasywny, to gra agresywnie tylko jeśli ma bardzo dobre karty. Jeśli natomiast jest agresywny, to z pewnym prawdopodobieństwem gra agresywnie nawet ze słabymi kartami. Informacją $I$, którą otrzymujemy może być agresywne rozegranie przez przeciwnika pewnej liczby rozdań. Wzór Bayesa pozwala stwierdzić jak bardzo informacja $I$ zwiększa prawdopodobieństwo tego, że mamy do czynienia z przeciwnikiem agresywnym.

Rozumowania tego typu nazywa się *wnioskowaniem Bayesowskim*.

---

## Metoda drzewkowa

Omówimy teraz tzw. "metodę drzewkową", której podstawę teoretyczną stanowią rozważania tego rozdziału. Moglibyśmy metodę tę omówić na przykładzie testu medycznego. Użyjemy jednak nieco innego przykładu, bardziej typowego zastosowania tej metody.  

**Przykład 2.9**  

Mamy dwie urny z kulami. W każdej z urn znajduje się pewna liczba kul białych i czarnych, znamy dokładnie zawartości obu urn. Wybieramy losowo urnę, a następnie z wybranej urny losujemy dwie kule. Jakie jest prawdopodobieństwo tego, że obie kule są czarne?

Zdefiniujmy następujące zdarzenia:

- $U_1,U_2$ - wybraliśmy odpowiednio urnę pierwszą/drugą,
- $B_1,C_1$ - pierwsza kula jest odpowiednio biała/czarna,
- $B_2,C_2$ - analogicznie dla drugiej kuli.

Rozwiązanie zadania mogłoby wyglądać tak:  

$P(B_1 \cap B_2) = P(U_1 \cap B_1 \cap B_2) + P(U_2 \cap B_1 \cap B_2) = P(U_1) P(B_1 | U_1) P(B_2 | U_1 \cap B_1) + P(U_2) P(B_1 | U_2) P(B_2 | U_2 \cap B_1).$  

Wszystkie prawdopodobieństwa występujące w ostatnim wyrażeniu są łatwe do obliczenia jeśli znamy zawartości urn.

W metodzie drzewkowej reprezentujemy wszystkie możliwe przebiegi losowania za pomocą drzewa.

TUTAJ OBRAZEK

Krawędzie na najwyższym poziomie odpowiadają wyborowi urny, krawędzie na drugim poziomie - losowaniu pierwszej kuli, krawędzie na trzecim poziomie - losowaniu drugiej kuli. Liczby na krawędziach są odpowiednimi prawdopodobieństwami warunkowymi. Jeśli na przykład wybraliśmy urnę pierwszą i wylosowaliśmy z niej kulę czarną ($U_1 \cap B_1$), to prawdopodobieństwo wylosowania kolejnej kuli czarnej, czyli przejścia do zdarzenia $U_1 \cap B_1 \cap B_2$ wynosi $P(B_2 | U_1 \cap B_1)$.  

W przypadku krawędzi wychodzących z korzenia mamy do czynienia z prawdopodobieństwami absolutnymi, choć oczywiście można uznać, że korzeń odpowiada zdarzeniu $\Omega$, i wtedy mamy $P(U_i|\Omega) = P(U_i)$.

Ze wzoru iloczynowego wynika, że prawdopodobieństwo znalezienia się w konkretnym wierzchołku jest równe iloczynowi liczb na ścieżce od korzenia do tego wierzchołka. W szczególności, aby znaleźć $P(B_1 \cap B_2)$ sumujemy $P(U_1 \cap B_1 \cap B_2)$ oraz $P(U_2 \cap B_1 \cap B_2)$ mnożąc liczby na odpowiednich ścieżkach.

Takie "graficzne" rozwiązanie wydaje się bardziej intuicyjne niż nasze pierwsze podejście sprowadzające się do przekształceń algebraicznych, i rzeczywiście często jest bardziej intuicyjne. Używając metody drzewkowej należy jednak być bardzo ostrożnym. Aby narysować drzewo scenariuszy często nie potrzebujemy formalnie definiować wszystkich istotnych zdarzeń (w praktyce często się tego nie robi). Zamiast tego zadowalamy się niezbyt formalnymi opisami sytuacji, którym odpowiadają poszczególne wierzchołki. Ten brak formalizmu często prowadzi do uznania rozwiązania błędnego (ew. rozwiązania innego problemu) za poprawne.

---

## Prawdopodobieństwo warunkowe jako metoda formułowania założeń

Naszą motywacją do zdefiniowania pojęcia prawdopodobieństwa warunkowego było nadanie sensu pytaniom takim jak w przykładzie 2.1. Okazało się jednak, że prawdopodobieństwo warunkowe może służyć także do formułowania założeń dotyczących zależności między zdarzeniami, np. wartości $P(T|C)$ w przykładzie 2.1, czy wartości $P(B_1|U_1)$ w przykładzie 2.9. To drugie zastosowanie pojęcia prawdopodobieństwa warunkowego jest o tyle interesujące, że pozwala ono często unikać formalnej definicji zbioru zdarzeń elementarnych $\Omega$. Zamiast tego postulujemy pewne zależności między prawdopodobieństwami interesujących nas zdarzeń. Tak właśnie postępujemy we wszystkich przykładach omówionych w tym rozdziale, z wyjątkiem przykładu 2.2.
