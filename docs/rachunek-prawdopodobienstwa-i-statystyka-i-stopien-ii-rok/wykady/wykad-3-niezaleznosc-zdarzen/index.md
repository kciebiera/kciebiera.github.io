---
title: "Wykład 3: Niezależność zdarzeń"
source_url: "http://smurf.mimuw.edu.pl/node/708"
source_kind: "html"
---

<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>

# Wykład 3: Niezależność zdarzeń

## Definicja niezależności

**Przykład 3.1**  

Przypomnijmy z poprzedniego wykładu przykład 2.2, w którym kolega zdradza nam sumę oczek z dwóch kostek. Przyjmijmy, że zamiast sumy pytamy go o wynik z pierwszej kostki i okazuje się, że było to 5. Jakie jest prawdopodobieństwo tego, że na drugiej kostce wypadło co najwyżej 2?

Do rozwiązania tego problemu moglibyśmy oczywiście użyć metody z poprzedniego wykładu, ale nie jest to konieczne. Wydaje nam się bowiem intuicyjnie oczywiste, że prawdopodobieństwo to jest równe $\frac{1}{3}$, tzn. jest dokładnie takie, jakie byłoby gdybyśmy w ogóle nie pytali.

Spróbujmy zbadać tę sytuację nieco bardziej formalnie. Jeśli przez $A$ oznaczymy zdarzenie odpowiadające wypadnięciu 5-ki na pierwszej kostce, a przez $B$ wypadnięciu co najwyżej 2-ki na drugiej kostce, to sądzimy, że powinno zachodzić:  

$P(B|A) = P(B)$,  

czyli  

$P(A \cap B) = P(A) P(B)$.  

Drugi z wzorów ma tę zaletę, że jest określony nawet jeśli $P(A)=0$ i to właśnie on jest podstawą definicji niezależności zdarzeń.

**Definicja (niezależność dwóch zdarzeń)**  

Zdarzenia $A,B \subseteq \Omega$ nazywamy *niezależnymi* jeśli $P(A \cap B) = P(A) P(B)$.

W sytuacji z przykładu 3.1 rozważane zdarzenia są niejako "fizycznie niezależne", opisują one wyniki rzutów dwiema różnymi kostkami. Należy jednak pamiętać, że zdarzenia mogą być niezależne w sensie powyższej definicji, nawet jeśli opisują wyniki tych samych eksperymentów. Dobrze pokazuje to poniższy przykład.

**Przykład 3.2**  

Rzucamy 3 razy monetą. Zdefiniujmy następujące zdarzenia:

- $A$ - wypadła co najwyżej jedna reszka,
- $B$ - wypadły i orły i reszki.

Mamy $P(A)= \frac{1}{2}$ (np. z symetrii) oraz $P(B) = \frac{3}{4}$ (bo z 8 zdarzeń elementarnych odrzucamy tylko 2). Ponadto $A \cap B$ odpowiada wypadnięciu dokładnie jednej reszki, więc $P(A \cap B) = \frac{3}{8}$. Czyli zachodzi $P(A \cap B) = P(A)P(B)$ pomimo tego, że zdarzenia $A$ i $B$ wydają się być od siebie zależne.

Ten przykład pokazuje, że o pojęciu niezależności należy myśleć w terminach informacji, t.j. "czy zajście zdarzenia $A$ daje mi dodatkową informację na temat zajścia zdarzenia $B$?", a nie w terminach związków fizycznych między zdarzeniami.

Na koniec naszych rozważań na temat definicji niezależności zauważmy następujący fakt:  

**Fakt 3.3**  

Jeśli zdarzenia $A,B$ są niezależne, to niezależne są też zdarzenia $A,\bar{B}$ oraz $\bar{A},\bar{B}$.  

**Dowód**  

Intuicyjnie powyższy fakt jest oczywisty. Jeśli zajście $B$ nie mówi nam nic o $A$, to nie zajście $B$ też nie powinno. Formalnie możemy rozumować tak:  

$P(A \cap \bar{B}) = P(A) - P(A \cap B) = P(A) - P(A)P(B) = P(A)(1-P(B)) = P(A) P(\bar{B}).$  

Druga część tezy wynika natychmiast z pierwszej zaaplikowanej do zdarzeń $\bar{B},A$.

---

## Niezależność więcej niż dwóch zdarzeń

Jak zdefiniować niezależność więcej niż dwóch zdarzeń? Naturalnym pomysłem jest zażądanie, aby każda para zdarzeń była niezależna, ale nie jest to pomysł szczególnie dobry, co ilustruje poniższy przykład.  

**Przykład 3.4**  

Rzucamy dwukrotnie monetą. Definiujemy następujące zdarzenia:

- $O_1$ - w pierwszym rzucie wypadł orzeł,
- $O_2$ - w drugim rzucie wypadł orzeł,
- $S$ - w obu rzutach wypadło to samo.

Łatwo sprawdzić, że $P(O_1) = P(O_2) = P(S) = \frac{1}{2}$.  

Ponadto $O_1 \cap O_2 = O_1 \cap S = O_2 \cap S$ i wspólne prawdopodobieństwo każdego z tych zbiorów wynosi $\frac{1}{4}$, a zatem każda para zdarzeń jest niezależna. Zauważmy jednak, że z podanej przez nas równości wynika, że zajście którychkolwiek dwóch zdarzeń implikuje zajście trzeciego. Intuicyjnie takie zdarzenia nie powinny być niezależne.

Aby poradzić sobie z sytuacjami takimi jak powyższa, definiujemy niezależność w nieco bardziej restrykcyjny sposób  

**Definicja (niezależność wielu zdarzeń)**  

Zdarzenia $A_1,A_2,\ldots,A_n$ nazywamy *niezależnymi* jeśli dla dowolnego ciągu indeksów $i_1 < i_2 < \ldots < i_k$ zachodzi  

$P(A_{i_1} \cap \ldots \cap A_{i_k}) = P(A_{i_1}) \cdot \ldots \cdot P(A_{i_k})$.

Tak jak w przypadku dwóch zdarzeń, w powyższym warunku możemy dowolną kombinację zdarzeń zastąpić przez ich dopełnienia i otrzymamy definicję równoważną:  

**Fakt 3.5**  

Niech $A_1,A_2,\ldots,A_n$ będą niezależne, i niech ponadto zdarzenia $B_1,\ldots,B_n$ będą takie, że dla każdego $i$ zachodzi $B_i = A_i$ lub $B_i = \bar{A_i}$. Wtedy zdarzenia $B_1,B_2,\ldots,B_n$ są niezależne.  

**Dowód**  

Przypadek $n=2$ rozważaliśmy wcześniej. Przypomnijmy, że najpierw pokazaliśmy tezę dla sytuacji, w której tylko jedno ze zdarzeń $B_i$ jest dopełnieniem odpowiedniego $A_i$, a następnie z tego wywnioskowaliśmy tezę dla sytuacji, gdy są dwa takie $B_i$.

Dowód w ogólnym przypadku jest analogiczny, ale musimy użyć indukcji po liczbie $B_i$ takich, że $B_i = \bar{A_i}$.  

Przyjmijmy, że dla każdego ciągu $i_1 < i_2 < \ldots < i_k$ takiego, że dla nie więcej niż $m$ indeksów $j$ zachodzi $B_{i_j} = \bar{A_{i_j}}$ pokazaliśmy już tożsamość  

$P(B_{i_1} \cap \ldots \cap B_{i_k}) = P(B_{i_1}) \cdot \ldots \cdot P(B_{i_k})$.  

Chcemy pokazać, że tożsamość ta zachodzi także dla ciągów, dla których równość $B_{i_j} = \bar{A_{i_j}}$ zachodzi dla $m+1$ indeksów.  

Bez straty ogólności przyjmijmy, że $i_j = j$ dla wszystkich $j = 1,\ldots,k$ oraz $B_k = \bar{A_k}$. Wtedy mamy:  

$P(B_1 \cap \ldots \cap B_k) = P(B_1 \cap \ldots \cap B_{k-1}) - P(B_1 \cap \ldots \cap B_{k-1} \cap A_k) = P(B_1)\cdot\ldots\cdot P(B_{k-1})(1-P(A_k)) = P(B_1)\cdot\ldots\cdot P(B_{k}) ,$  

gdzie dwukrotnie korzystamy z założenia indukcyjnego.

Nasz nieudany pomysł na definicję niezależności też ma swoją nazwę i miejsce w teorii prawdopodobieństwa:  

**Definicja (niezależność parami)**  

Zdarzenia $A_1,A_2,\ldots,A_n$ nazywamy *parami niezależnymi* jeśli dla każde dwa zdarzenia spośród $A_1,A_2,\ldots,A_n$ są niezależne.

W sytuacji j.w. mówi się też czasem o  *2-niezależności*, i ogólniej o *k-niezależności*, jeśli niezależny jest każdy podzbiór $k$ zdarzeń.

---

## Niezależność jako założenie

Po co definiujemy niezależność? Doświadczony probabilista byłby zapewne w stanie omawiać to zagadnienie całymi godzinami. W przypadku naszego elementarnego wykładu odpowiedź jest jednak dość prosta. Niezależność z jednej strony często występuje w interesujących nas doświadczeniach losowych, z drugiej ma bardzo prosty opis w naszej teorii i niezmiernie upraszcza wiele rozważań. W związku z tym niezależność jest bardzo wygodnym *założeniem*.

Jeśli na przykład wykonujemy 3 rzuty kostką, to sensownie jest przyjąć, że zdarzenia odpowiadające wynikom z różnych kostek są niezależne. Ale skoro tak, to obliczanie prawdopodobieństwa dowolnych zdarzeń można, korzystając z niezależności, sprowadzić do obliczania zdarzeń dotyczących jednej kostki.

I ogólniej: jeśli wykonujemy szereg doświadczeń i sądzimy, że zdarzenia odpowiadające ich wynikom są niezależne, to możemy obliczanie prawdopodobieństw dowolnych zdarzeń sprowadzić do obliczania zdarzeń dotyczących pojedynczych doświadczeń.

W tym sensie można myśleć o niezależności  (podobnie jak prawdopodobieństwie warunkowym) jako o metodzie budowania modelu probabilistycznego. Metoda ta polega na odpowiednim komponowaniu mniejszych modeli odpowiadających niezależnym aspektom modelowanego zjawiska.

---

## Przestrzeń produktowa (dla chętnych)

Rozważania z poprzedniego paragrafu można sformalizować. Choć nie będziemy nigdy wprost z korzystać z rozważań, które za chwilę przeprowadzimy, implicite będą się one pojawiać wielokrotnie. Dlatego warto się z nimi zapoznać pomimo tego, że są one nieco bardziej wymagające matematycznie.

Rozważmy dwa doświadczenia $D_1, D_2$ takie, że zdarzenia odpowiadające wynikom $D_1$ są niezależne od zdarzeń odpowiadających wynikom $D_2$. Dla uproszczenia będziemy w takiej sytuacji po prostu mówić, że same doświadczenia są niezależne. Przyjmijmy, że wynik doświadczenia $D_1$ jest modelowany przestrzenią $(\Omega_1,P_1)$, a wynik $D_2$ przestrzenią $(\Omega_2,P_2)$. Jak zdefiniować przestrzeń $(\Omega,P)$ dla doświadczenia $D$  polegającego na niezależnym wykonaniu $D_1$ i $D_2$? Na pewno $\Omega=\Omega_1\times \Omega_2$, ale jak zdefiniować P? Powinniśmy zagwarantować, że dla każdego $A_1, A_2$ zachodzi  $P(A_1 \times A_2) = P(A_1 \times \Omega_2) P(\Omega_1 \times A_2) = P_1(A_1)P_2(A_2)$. Czy da się takie $P$ znaleźć?

Jeśli $\Omega_1$ i $\Omega_2$ są przeliczalne, lub ogólniej jeśli $P_1$ i $P_2$ są skoncentrowane na przeliczalnej liczbie elementów, to wystarczy zdefiniować $P$ na singletonach wzorem $P( (x_1,x_2) ) = P_1(x_1)P_2(x_2)$, a następnie rozszerzyć na wszystkie zdarzenia sumując odpowiednie singletony. Łatwo sprawdzić, że tak zdefiniowane $P$ spełnia nasze postulaty. W ogólnym przypadku musimy się odwołać do odpowiedniego twierdzenia z analizy, które gwarantuje istnienie, dla dowolnych dwóch miar $P_1,P_2$ tzw. miary produktowej, która okazuje się być miarą $P$, której szukamy.

Co ważne, twierdzenie o istnieniu miary produktowej zachodzi także dla przeliczalnie wielu miar. Możemy zatem z niego skorzystać do konstrukcji przestrzeni probabilistycznej opisującej łączny wynik przeliczalnie wielu niezależnych doświadczeń. Do czego może nam się to przydać? Oto przykład: Wiele doświadczeń dotyczących rzutów monetą (na przykład rzucanie do k-tego orła, itp.) wygodnie jest opisywać zakładając, że rzucamy monetą "w nieskończoność". Łatwo jest zdefiniować model probabilistyczny pojedynczego rzutu monetą. A jak zdefiniować model nieskończonego ciągu rzutów? Łatwo zauważyć, że zadanie to nie jest proste, choćby dlatego, że każdy konkretny ciąg wyników musi mieć prawdopodobieństwo 0. Z pomocą, tak jak poprzednio, przychodzi nam teoria miary, która gwarantuje istnienie odpowiedniej miary produktowej.

Powyższą dyskusję należy traktować jako nieco bardziej formalne uzasadnienie tego, że postulaty w rodzaju "Ponieważ te dwa zdarzenia powinny być niezależne, postuluję aby $P(A \cap B) = P(A)P(B)$" dają się zrealizować na gruncie naszej teorii.
