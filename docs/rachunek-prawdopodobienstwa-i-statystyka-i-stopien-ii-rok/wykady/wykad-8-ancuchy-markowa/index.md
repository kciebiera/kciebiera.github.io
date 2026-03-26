---
title: "Wykład 8: Łańcuchy Markowa"
source_url: "http://smurf.mimuw.edu.pl/node/713"
source_kind: "html"
---

<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@4/tex-mml-chtml.js"></script>

# Wykład 8: Łańcuchy Markowa

## Motywacja

Zanim zdefiniujemy pojęcie łańcucha Markowa przyjrzyjmy się przykładom sytuacji, które można za ich pomocą opisywać i pytań, na które pozwalają one odpowiadać.

**Przykład 8.1 (Student walczy)**  

Student raz w tygodni bierze udział w zajęciach z rachunku prawdopodobieństwa. Na każde zajęcia przychodzi przygotowany bądź nie. Jeśli w danym tygodniu jest przygotowany, to w następnym jest przygotowany z prawdopodobieństwem 0.7. Jeśli natomiast w danym tygodniu nie jest przygotowany, to w następnym jest przygotowany z prawdopodobieństwem 0.2. Interesują nas odpowiedzi na pytania w rodzaju:

- Jeśli student jest w tym tygodniu nieprzygotowany, to ile tygodni musimy średnio czekać aż będzie przygotowany?
- Na dłuższą metę, jak często student jest przygotowany?

**Przykład 8.2 (Turniej trzyosobowy)**  

Trzech graczy: A, B i C gra turniej systemem "przegrywający odpada". Turniej zaczyna się od pojedynku A z B, a w każdej kolejnej grze gra zawodnik, który w poprzedniej nie grał ze zwycięzcą. Dla każdej pary zawodników znamy ich prawdopodobieństwa wygrania bezpośredniego pojedynku.

- Jak długo będziemy średnio czekać na drugi pojedynek A z B?
- Jeśli rozgrywany turniej będzie się składał z bardzo wielu gier, to średnio jaką część wszystkich gier rozegranych będą grać między sobą gracze A i B? Jak wiele gier (spośród wszystkich gier) wygra średnio gracz A?

**Przykład 8.3 (Hazardzista)**  

Hazardzista zaczyna grać z kapitałem początkowym 1000 zł. W każdej rundzie rozgrywki z prawdopodobieństwem 0.5 wygrywa 10 zł i z prawdopodobieństwem 0.5 przegrywa 10zł. Celem hazardzisty jest zdobycie kwoty 5000zł, ale zakończy grę także jeśli wcześniej zbankrutuje.

- Jakie jest prawdopodobieństwo, że uda mu się zdobyć 5000zł?
- Jak długo musi średnio grać, aby zdobyć tę kwotę lub zbankrutować?
- Co jeśli hazardzista nie przestaje grać, chyba że zbankrutuje? Jakie jest prawdopodobieństwo tego, że się to stanie? (ignorujemy w tym miejscu, to że wcześniej umrze/zachoruje/zaśnie/inne)

---

## Definicja łańcucha Markowa

Zauważmy, że we wszystkich powyższych przykładach mamy do czynienia z obiektem/systemem/układem, który zawsze znajduje się w jednym z pewnej liczby stanów (student jest w stanie "przygotowany", bądź w stanie "nieprzygotowany", hazardzista w stanie "0zł", "1zł", itd.). Ponadto stan, w którym znajdzie się za chwilę jest wybierany losowo, ale prawdopodobieństwa znalezienia się w poszczególnych stanach zależą tylko od aktualnego stanu.

Formalnie taką sytuację możemy modelować następująco:  

**Definicja (łańcuch Markowa)**  

Łańcuchem Markowa o zbiorze stanów $S \subseteq \mathbb{R}$ nazywamy ciąg zmiennych losowych $X_0,X_1,X_2,\ldots$ taki, że  

$P(X_n = x_n | X_{n-1} = x_{n-1} \wedge X_{n-2} = x_{n-2} \wedge \ldots \wedge X_0 = x_0 ) = P(X_n = x_n | X_{n-1} = x_{n-1}) = p_{x_{n-1},x_n}$ dla każdego $n > 0$ i ciągu stanów $x_0,x_1,\ldots,x_n \in S$.

W tej definicji zmienna $X_t$ opisuje stan, w którym znajduje się łańcuch Markowa w chwili $t$. Intuicyjnie, warunek z definicji łańcucha Markowa mówi, że zmienna $X_t$ zależy tylko od zmiennej $X_{t-1}$. Co ważne, nie znaczy to, że $X_t$ jest niezależna od $X_0,\ldots,x_{t-2}$. Znaczy to jedynie tyle, że "cała zależność $X_t$ od $X_0,\ldots,X_{t-1}$ jest zawarta w zależności $X_t$ od $X_{t-1}$".

**Uwaga 8.4**  

Ze względu na definicję zmiennej losowej (przypomnijmy: zmienne losowe mają wartości rzeczywiste), w powyższej definicji zakładamy, że stany łańcucha Markowa są liczbami rzeczywistymi. W praktyce takie ograniczenie jest dość niewygodne, np. w przykładzie z trzyosobowym turniejem naturalne byłoby przyjęcie $S = \{\{A,B\},\{B,C\},\{A,C\}\}$. Zawsze można jednak przypisać takim ogólniejszym stanom etykiety będące liczbami rzeczywistymi, czy nawet naturalnymi, i w ten sposób uzyskać łańcuch Markowa zgodny z definicją.

Liczby $p_{i,j}$ występujące w definicji łańcucha Markowa to tzw. prawdopodobieństwa przejścia w jednym kroku. Jeśli w danej chwili łańcuch jest w stanie $i$, to z prawdopodobieństwem $p_{i,j}$ znajdzie się w następnej chwili w stanie $j$. Liczby te wygodnie jest ułożyć w macierz:  

**Definicja (macierz przejścia)**  

*Macierzą przejścia (w jednym kroku)* łańcucha Markowa nazywamy macierz $M = (p_{i,j})_{ i,j \in S}$.

**Uwaga**  

W dalszej części wykładu będziemy symbolem $M$ oznaczać zarówno macierz przejścia łańcucha, jak i sam łańcuch.

Z oczywistych względów zachodzi następujący fakt.  

**Fakt 8.5**  

Wiersze macierzy przejścia łańcucha Markowa sumują się do 1.

**Uwaga 8.6**  

Może się zdarzyć, że zbiór $S$ nie będzie skończony (może nawet nie być przeliczalny, ale takie sytuacje nie będą nas interesować). W związku z tym zdefiniowany powyżej obiekt może czasem nie być (skończoną) macierzą. Jak jednak łatwo zauważyć podstawowe operacje macierzowe takie jak mnożenie macierzy przez wektor, czy przez inną macierz, mają sens także dla takich "nieskończonych macierzy". W ogólnym przypadku mogą pojawić się problemy związane ze zbieżnością, ale w naszym przypadku problemy takie nie wystąpią ze względu na Fakt 8.5. W związku z tym, w dalszym ciągu wykładu będziemy ignorowali tę kwestię i mówili po prostu o macierzy przejścia.

**Przykład 8.1 (Student walczy, c.d.)**  

W przykładzie ze studentem mamy dwa stany: 1 - student jest przygotowany, oraz 2 - student nie jest przygotowany. Macierz przejścia wygląda w tym przypadku tak:  

$\left(  

\begin{array}{cc}  

0.7 & 0.3 \\  

0.2 & 0.8  

\end{array}  

\right)$

Przypuśćmy, że znamy rozkład zmiennej $X_t$, tzn. prawdopodobieństwa tego, że w chwili $t$ znajdujemy się w poszczególnych stanach. Jak wygląda rozkład zmiennej $X_{t+1}$? I ogólniej, jak wygląda rozkład zmiennej $X_{t+s}$ dla $s \in \mathbb{N}$ ? Okazuje się, że można go łatwo obliczyć korzystając z macierzy $M$.  

**Fakt 8.7**  

Niech $\pi(t)$ będzie rozkładem $X_t$ reprezentowanym za pomocą wierszowego wektora prawdopodobieństw poszczególnych stanów, t.j.\ $\pi(t)_s = P(X_t = s)$.  Wtedy  

$\pi(t+1) = \pi(t) M$  

i ogólniej  

$\pi(t+s) = \pi(t) M^s$.  

**Dowód**  

Ze wzoru na prawdopodobieństwo całkowite mamy:  

$P(X_{t+1} = a) = \sum_{b \in S} P(X_t = b) P(X_{t+1} = a | X_t = b) = \sum_{b \in S} \pi(t)_b M_{b,a}$,  

a to jest definicja mnożenia macierzy przez wektor (z lewej strony!).  

Ogólniejsza wersja wynika z prostszej przez indukcję.

Powyższy fakt pokazuje siłę macierzowej reprezentacji łańcuchów Markowa. Istnieją bowiem algorytmy pozwalające bardzo szybko obliczyć $M^s$ nawet dla dużych $s$ (oparte, na przykład, na rozkładzie Jordana macierzy). Dzięki temu możemy w relatywnie prosty sposób obliczać rozkład $X_{t+s}$ znając rozkład $X_t$.

W dalszej części wykładu elementy macierzy $M^k$ będziemy oznaczać $p_{i,j}(k)$. W szczególności $p_{i,j} = p_{i,j}(1)$.

---

## Prawdopodobieństwa dotarcia

Zastanowimy się teraz jak można odpowiadać na pytania w rodzaju "Jeśli łańcuch Markowa jest w stanie $b$, to jakie jest prawdopodobieństwo tego, że kiedykolwiek dotrze do stanu $a$?". Będziemy to prawdopodobieństwo oznaczać $f_{b,a}$.  Symbol $f_{a,a}$ będziemy interpretować jako prawdopodobieństwo tego, że wrócimy do stanu $a$ po wykonaniu co najmniej jednego kroku. Innym naturalnym pomysłem byłoby przyjęcie $f_{a,a} = 1$, ale wybrana przez nas konwencja jest wygodniejsza w praktyce (choć prowadzi do nieco bardziej skomplikowanych wzorów).

**Twierdzenie 8.8**  

Niech $M$ będzie łańcuchem Markowa o zbiorze stanów $S$ i niech $a,b \in S$. Wtedy  

$f_{b,a} = \sum_{c \in S \setminus a} p_{b,c} f_{c,a} + p_{b,a}$.

**Dowód**  

Przyjmijmy, że $X_0 = b$ i niech $A$ będzie zdarzeniem odpowiadającym dotarciu kiedykolwiek do stanu $a$, formalnie $A = \{ \omega \in \Omega : \exists_{i \ge 1} X_i(\omega) = a \}$.  

Interesuje nas obliczenie P(A).  

Ze wzoru na prawdopodobieństwo całkowite mamy  

$P(A) = \sum_{c \in S} P(X_1 = c) P(A|X_1 =c) = \sum_{c \in S} p_{b,c} P(A|X_1 = c)$.  

Łatwo zauważyć, że $P(A | X_1 = c) = f_{c,a}$ dla $c \neq a$ (zachęcamy czytelnika do uzasadnienia tego formalnie) oraz $P(A|X_1 = a) = 1$, skąd dostajemy tezę.

---

## Klasyfikacja stanów

Przypomnijmy, że symbolem $p_{i,j}(k)$ oznaczamy prawdopodobieństwo przejścia ze stanu $i$ do stanu $j$ w $k$ krokach, a symbolem $f_{i,j}$ prawdopodobieństwo dotarcia (kiedykolwiek) z $i$ do $j$. Niech ponadto $f_{i,j}(k)$ będzie prawdopodobieństwem dotarcia z $i$ do $j$ po raz pierwszy po $k$ krokach. Oczywiście zachodzi $f_{i,j} = \sum_{k=1}^\infty f_{i,j}(k)$.

**Definicja (stan powracający/chwilowy)**  

Stan $a$ jest *powracający*, jeśli $f_{a,a} = 1$, czyli jeśli wracamy do niego z prawdopodobieństwem $1$. Stan jest *chwilowy*, jeśli nie jest powracający.

**Przykład 8.9**  

Łatwo zauważyć oba stany studenta i wszystkie trzy stany turnieju trzyosobowego są powracające. W przykładzie z hazardzistą bankructwo i osiągniecie celu są stanami powracającymi, natomiast wszystkie pozostałe stany są chwilowe.

**Twierdzenie 8.10**  

Stan $a$ jest powracający wtedy i tylko wtedy, gdy $\sum_{k \ge 1} p_{a,a}(k) = \infty$.

**Uwaga 8.11**  

Tezę powyższego twierdzenia można interpretować następująco: stan $a$ jest powracający wtedy i tylko wtedy, gdy średnia liczba powrotów do $a$ jest nieskończona. Jest to interpretacja bardzo przydatna, ale niestety nie do końca poprawna - liczba powrotów może być nieskończona, a zatem nie jest zmienną losową w sensie naszej definicji.

**Dowód**  

Aby uniknąć problemu, o którym wspomnieliśmy w uwadze 8.11, przyjmijmy, że $X_0 = a$ oraz niech  $Y_N$ będzie liczbą powrotów do $a$ w pierwszych $N$ krokach (ograniczając się do pierwszych $N$ kroków unikamy nieskończonych wartości). Wtedy z liniowości wartości oczekiwanej  

$EY_N = \sum_{k=1}^N E[X_k=a] = \sum_{k=1}^N P(X_k=a) = \sum_{k=1}^N p_{a,a}(k)$,  

a zatem $\sum_{k \ge 1} p_{a,a}(k) = \infty$ jest granicą $EY_N$.

Jeśli $a$ jest stanem powracającym, to $\sum_{k\ge 1} f_{a,a}(k) = 1$, a zatem istnieje $K$ takie, że $\sum_{k=1}^K f_{a,a}(k) \ge 1-\epsilon$, tzn.\ z prawdopodobieństwem $\ge 1-\epsilon$ wrócimy do stanu $a$ w nie więcej niż $K$ krokach. Mamy wtedy $P(Y_K \ge 1) \ge 1-\epsilon$, a także $P(Y_{2K} \ge 2) \ge P(Y_K \ge 1)^2 \ge (1-\epsilon)^2$ i ogólnie $P(Y_{nK} \ge n) \ge (1-\epsilon)^n$.  

Dla $N \ge nK$ dostajemy  

$EY_N = \sum_{i = 1}^\infty P(Y_N \ge i) \ge \sum_{i=1}^n P(Y_N \ge i) \ge \sum_{i=1}^n P(Y_{iK} \ge i) \ge \sum_{i=1}^n (1-\epsilon)^i = \frac{1-(1-\epsilon)^{n+1}}{\epsilon}$.  

Biorąc odpowiednio małe $\epsilon$ i odpowiednio duże $n$ (a zatem także odpowiednio duże $N$) widzimy, że $EY_N$ może przyjmować dowolnie duże wartości, a co za tym idzie $\lim_{N \rightarrow \infty} EY_N = \infty$.

Aby pokazać implikację w drugą stronę, przyjmijmy, że $a$ jest chwilowy. Wtedy $f_{a,a} < 1$ . Dla dowolnego $N$ zachodzi $P(Y_N \ge k) \le f_{a,a}^k$, a zatem $EY_N = \sum_{k=1}^\infty P(Y_N \ge k) \le \frac{f_{a,a}}{1-f_{a,a}}$, co kończy dowód.

**Definicja (stany osiągalne)**  

Stan $b$ jest *osiągalny* ze stanu $a$ jeśli istnieje takie $k \in \mathbb{N}$, że $p_{a,b}(k) > 0$. Zbiór stanów osiągalnych ze stanu $a$ oznaczamy $A(a)$.

**Definicja (stany komunikujące się)**  

Stany $a,b$ komunikują się jeśli $a \in A(b)$ oraz $b \in A(a)$.

**Uwaga 8.12**  

Te definicje mają naturalną interpretację grafową. Jeśli zbudujemy graf, w którym wierzchołkami są stany łańcucha, a krawędź skierowana $(a,b)$ istnieje, gdy $p_{a,b} > 0$, to:

- $b$ jest osiągalny z $a$ jeśli istnieje skierowana ścieżka z $a$ do $b$,
- $a$ i $b$ komunikują się, jeśli są w tej samej silnie spójnej składowej.

**Fakt 8.13**  

Jeśli stan $a$ jest powracający, to każdy stan $b$ osiągalny z $a$ też jest powracający i zachodzi $A(a) = A(b)$.  

**Dowód (szkic)**  

Ponieważ $b$ jest osiągalny z $a$ to dla pewnego $k$ zachodzi $p = p_{a,b}(k) > 0$. Z drugiej strony, ponieważ $a$ jest powracający, to z prawdopodobieństwem $1$ wrócimy do $a$ po $k$-tym kroku. Stąd $f_{b,a} = 1$. Niech $X_0 = b$ i niech $Y_N$ i $Z_N$ będą liczbą odwiedzin odpowiednio $a$ i $b$ w pierwszych $N$ krokach. Wtedy wiemy z twierdzenia 8.10, że $\lim_{N\rightarrow\infty}EY_N = \infty$, ale ponieważ $EZ_{N+k} \ge pEY_N$ (bo za każdym razem gdy odwiedzamy $a$ mamy szansę $p$ na odwiedzenie $b$ po $k$ krokach), to dostajemy też $\lim_{N\rightarrow \infty} EZ_N = \infty$, co w połączeniu z twierdzeniem 8.10 oznacza, że $b$ jest powracający. Druga część tezy jest oczywista.  

(Zachęcamy czytelnika do uzupełnienia  powyższej argumentacji.)

Z powyższych rozważań wynika następujący wniosek, bardzo przydatny w szybkiej klasyfikacji stanów łańcucha Markowa.  

**Wniosek 8.14**  

Niech $G$ będzie grafem odpowiadającym skończonemu łańcuchowi Markowa $M$, jak w uwadze 8.12 i niech $D$ będzie DAG-iem silnie spójnych składowych $G$. Wtedy:

- każda silnie spójna składowa (zwana często klasą) składa się albo z samym stanów powracających (tzw. klasy powracające), albo z samych stanów chwilowych (tzw. klasy chwilowe),
- ujścia $D$ (czyli wierzchołki, z których nie wychodzą żadne krawędzie) odpowiadają klasom powracającym, a pozostałe wierzchołki klasom chwilowym.

**Wniosek 8.15**  

W skończonym łańcuchu Markowa dla każdego stanu $a$ istnieje stan powracający $b$ osiągalny z $a$. W szczególności każdy skończony łańcuch Markowa zawiera stan powracający.

**Definicja (łańcuch nieredukowalny)**  

Łańcuch nazywamy *nieredukowalnym* (w starszej literaturze *nieprzywiedlny*), jeśli składa się z jednej klasy powracającej.

**Uwaga 8.16**  

Z wniosku 8.15 wynika, że w przypadku łańcuchów skończonych nie ma potrzeby żądać, by jedyna klasa łańcucha była powracająca - ona musi być powracająca bo zawiera stan powracający. Nie jest to jednak prawdą w przypadku łańcuchów nieskończonych.

---

## Średnie czasy dotarcia

Jesteśmy już gotowi odpowiedzieć na pytania postaci "Średnio po ilu krokach łańcuch Markowa dotrze do stanu $a$?". Ograniczymy się przy tym do następującej sytuacji: Niech $M$ będzie skończonym łańcuchem Markowa o dokładnie jednej klasie powracającej i niech $a$ będzie jednym z jej elementów. Wtedy dla każdego stanu $b$ zachodzi $f_{b,a}=1$ (dlaczego?). Interesuje nas obliczenie dla każdego stanu $b$ średniej liczby kroków $\mu_{b,a}$ potrzebnej na dotarcie do $a$. W szczególności przez $\mu_{a} = \mu_{a,a}$ oznaczamy średnią liczbę kroków potrzebnych, aby wrócić do $a$.

**Twierdzenie 8.17**  

Niech $M$ będzie jak powyżej. Wtedy dla każdego $b \in S$ zachodzi:  

$\mu_{b,a} = \sum_{c \in S \setminus a} p_{b,c}(1+\mu_{c,a}) + p_{b,a} = 1+\sum_{c \in S \setminus a} p_{b,c} \mu_{c,a}$.

**Uwaga 8.18**  

Jeśli w łańcuchu istnieje więcej niż jeden stan powracający, a interesuje nas dojście do któregokolwiek z nich, to możemy skleić wszystkie stany powracające w jeden (zachęcamy czytelnika do uzupełnienia szczegółów). Takie sklejenie można wykonać nawet jeśli istnieje więcej niż jedna klasa powracająca.  

W takim przypadku można też próbować obliczać średni czas dojścia do stanu chwilowego pod warunkiem, że do niego w ogóle dojdziemy. Ta sytuacja jest bardziej skomplikowana i nie będziemy się nią tu zajmować.

**Dowód**  

Dowód przebiega analogicznie do dowodu twierdzenia 8.8.  

Załóżmy, że $X_0 = b$ i niech $\tau = \min_t \{ X_t = a \}$, czyli $\tau$ jest liczbą kroków, po której po raz pierwszy docieramy do stanu $a$. Interesuje nas obliczenie  

$\mu_{b,a} = E(\tau)$.  

Ze wzoru na całkowitą wartość oczekiwaną mamy  

$\mu_{b,a} = \sum_{c \in S} P(X_1 = c) E(\tau | X_1 = c) = \sum_{c \in S} p_{b,c} E(\tau | X_1 = c)$.  

Łatwo zauważyć, że $E(\tau | X_1 = c) = \mu_{c,a}+1$ dla $c \neq a$(zachęcamy czytelnika do uzasadnienia tego formalnie) oraz $E(\tau|X_1 = a) = 1$, skąd dostajemy tezę.

---

## Rozkład stacjonarny i długookresowe zachowanie łańcucha Markowa

Postaramy się teraz opisać długookresowe zachowanie łańcucha Markowa. Dzięki temu będziemy odpowiadać na pytania w rodzaju:

- Na dłuższą metę, jak często student jest przygotowany? (patrz przykład 8.1)
- Jeśli rozgrywamy bardzo dużo gier, to jak często grają ze sobą gracze A i B? (patrz przykład 8.2)

Żeby to zrobić, musimy najpierw wprowadzić kilka definicji.

**Definicja (okresowość)**  

Stan $a$ jest okresowy, jeśli istnieje liczba całkowita $d > 1$ (zwana okresem $a$) taka, że jeśli $p_{a,a}(k) > 0$  dla pewnego $k$, to $d | k$. Stan, który nie jest okresowy nazywamy *nieokresowym*. Łańcuch Markowa nie zawierający stanów okresowych nazywamy *łańcuchem nieokresowym*  

**Przykład 8.19**  

Rozważmy łańcuch Markowa $M$ o stanach $0,1,\ldots,k-1$ i przejściach $p_{a,b} = 1$ jeśli $b = (a+1) \mod k$ oraz $p_{a,b} = 0$ wpp. Wtedy wszystkie stany $M$ mają okres $k$.

**Fakt 8.20**  

Jeśli $a$ jest okresowy z okresem $d$ to każdy stan $b$ komunikujący się z $a$ też jest okresowy z okresem $d$.  

**Dowód**  

Ponieważ $a$ i $b$ się komunikują, to mamy $p_{a,b}(k) > 0$ oraz $p_{b,a}(l) > 0$ dla pewnych $k,l > 0$. Gdyby $b$ nie spełniał tezy, to istniałoby $i > 0$ niepodzielne przez $d$ i takie, że $p_{b,b}(i) > 0$. Ale wtedy moglibyśmy przejść z $a$ do $a$ zarówno w $k+l$ krokach jak i $k+l+i$ krokach, co nie jest możliwe, bo co najmniej jedna z tych liczb nie jest podzielna przez $d$.

Możemy już sformułować najważniejszy wynik tego rozdziału:  

**Twierdzenie 8.21 (ergodyczne)**  

Dla  skończonego nieredukowalnego i nieokresowego łańcucha Markowa $M$ o zbiorze stanów $S$ istnieje wektor $\pi_a, a \in S$ takie, że:

1. $\sum_{a \in S} \pi_a = 1$ oraz $0 \le \pi_a \le 1$ dla wszystkich $a \in S$,
2. $\pi M = \pi$,
3. dla każdego $a,b \in S$ zachodzi $\lim_{n \rightarrow \infty} p_{a,b}(n) = \pi_{b}$.

Pojawiający się w powyższym twierdzeniu wektor $\pi$ nazywa się z reguły *rozkładem stacjonarnym*, co można łatwo zrozumieć patrząc na pierwsze dwa punkty tezy twierdzenia ergodycznego. Pierwszy z punktów mówi, że $\pi$ jest rozkładem prawdopodobieństwa na zbiorze stanów $S$. Drugi punkt mówi, że jest on stacjonarny w następującym sensie: Jeśli $X_t$ ma rozkład $\pi$, to $X_{t+1}$ także ma rozkład $\pi$.

Najważniejszy jest oczywiście punkt trzeci, który mówi, że niezależnie od tego w jakim stanie łańcuch startuje, po odpowiednio długim czasie zbiegnie do rozkładu $\pi$. Innymi słowy, niezależnie od rozkładu $X_0$, dla odpowiednio dużych $t$, zmienna $X_t$ będzie miała rozkład dowolnie bliski $\pi$.

**Przykład 8.22**  

Widać, że twierdzenie ergodyczne jest dokładnie tym czego potrzebujemy, aby móc odpowiadać na pytania, które zadaliśmy na początku tego rozdziału, np. "Na dłuższą metę, jak często student jest przygotowany?". Przypomnijmy, że student znajduje się w jednym z dwóch stanów: 1 - przygotowany, 2 - nieprzygotowany. Macierz przejścia między tymi stanami wygląda następująco:  

$M =  

\left(  

\begin{array}{cc}  

0.7 & 0.3 \\  

0.2 & 0.8  

\end{array}  

\right)$  

Odpowiedzią na pytanie, o to jak często student jest na dłuższą metę przygotowany jest liczba $\pi_1$ z twierdzenia ergodycznego. Jak ją znaleźć?  Oczywiście za pomocą twierdzenia ergodycznego, a konkretnie dwóch pierwszych punktów tezy tego twierdzenia. Mamy mianowicie:  

$\pi M = \pi$,  

czyli  

$0.7\pi_1 + 0.2 \pi_2 = \pi_1$ ,  

$0.3\pi_1 + 0.8 \pi_2 = \pi_2$  

(tak naprawdę wystarczy jedno z tych równań, dlaczego?)  

oraz  

$\pi_1 + \pi_2 = 1$.  

Łatwo dostajemy $\pi_1 = 0.4$, $\pi_2 = 0.6$.

**Uwaga 8.23**  

Łatwo zauważyć, że założenia twierdzenia ergodycznego są do pewnego stopnia konieczne:

- Jeśli nasz łańcuch jest okresowy, na przykład jest cyklem, i wystartujemy z jednego z punktów tego cyklu, to w dowolnym kroku będziemy w jednym punkcie cyklu, w szczególności nie może być mowy o zbieżności do jakiegokolwiek stabilnego rozkładu.
- Założenie, że łańcuch jest nieredukowalny można nieco osłabić. Wystarczy, że zawiera dokładnie jedną powracającą klasę stanów. Twierdzenie zachodzi w takiej sytuacji w zasadzie bez zmian, a wszystkie stany chwilowe mają w rozkładzie stacjonarnym prawdopodobieństwo zerowe. Jeśli jednak nasz łańcuch zawiera więcej niż jedną powracającą klasę stanów to twierdzenie nie zachodzi. W zależności od tego, gdzie wystartujemy, uzyskamy inny rozkład stacjonarny. W szczególności, jeśli wystartujemy w którejś z klas powracających, to nigdy z niej nie wyjdziemy.
- Twierdzenie w postaci podanej wyżej nie zachodzi dla łańcuchów nieskończonych. Przykładem nieskończonego nieredukowalnego i nieokresowego łańcucha, który nie ma rozkładu stacjonarnego jest błądzenie losowe po prostej (dowód wykracza poza zakres tego wykładu). Aby teza twierdzenia zachodziła należy dodatkowo zażądać, aby wszystkie powracające stany łańcucha były *niezerowe*, tzn. aby dla każdego stanu powracającego $a$ zachodziło $\mu_{a,a} < \infty$.

Ostatnie twierdzenie tego wykładu pokazuje interesujące zastosowanie rozkładu stacjonarnego.  

**Twierdzenie 8.24**  

Przy założeniach jak w twierdzeniu ergodycznym dla każdego stanu $a \in S$ zachodzi $\mu_{a,a} = \frac{1}{\pi_a}$.

Formalny dowód pominiemy, ale intuicyjnie twierdzenie to powinno być jasne: Skoro na dłuższą metę $\pi_a$ jest frakcją kroków, w których łańcuch jest w stanie $a$, to średni odstęp między wystąpieniami stanu $a$ wynosi $\frac{1}{\pi_a}$.

**Przykład 8.25**  

Możemy z tego twierdzenia wywnioskować, że jeśli student jest w danym tygodniu przygotowany, to musimy czekać średnio $\frac{1}{0.4} = 2.5$ tygodnie, aż znów będzie przygotowany. Akurat w tym przypadku ten sam wynik można w bardzo prosty sposób uzyskać wprost z definicji, ale jeśli łańcuch jest bardziej skomplikowany twierdzenie ergodyczne jest często najprostszą metodą.

---

## Zastosowanie łańcuchów Markowa

Łańcuchy Markowa mają mnóstwo bardzo różnorodnych zastosowań. Wspomnimy w tym miejscu o zaledwie kilku z nich:

- Losowanie obiektów: Aby wygenerować losowy obiekt zgodnie z pewnym rozkładem konstruujemy łańcuch Markowa dla którego rozkład ten jest rozkładem stacjonarnym, po czym wykonujemy odpowiednio długie symulacje tego łańcucha. To podejście nazywa się metodą Monte Carlo z wykorzystaniem łańcuchów Markowa (ang. Markov Chain Monte Carlo).
- Modelowanie: Za pomocą łańcuchów Markowa można skutecznie modelować wiele naturalnych procesów i struktur. Na przykład modelując w ten sposób język naturalny można zbudować algorytm kompresji tekstu. Alternatywnie, modeli takich można użyć do generowania losowych tekstów. Modele Markowa pojawiają się też bardzo często w biologii obliczeniowej.
- PageRank: Imponującym zastosowaniem łańcuchów Markowa jest stworzony przez firmę Google algorytm szeregowania stron PageRank. Algorytm ten bazuje na łańcuchu Markowa, który jest modelem procesu poruszania się użytkownika po zbiorze wszystkich (znanych systemowi) stron WWW.
