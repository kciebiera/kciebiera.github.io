---
title: "Wykład 4: Dyskretne zmienne losowe"
source_url: "http://smurf.mimuw.edu.pl/node/709"
source_kind: "html"
---

<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>

# Wykład 4: Dyskretne zmienne losowe

## Motywacja i definicja zmiennej losowej

### Motywacja

Dotychczas nie nakładaliśmy żadnych warunków na to jak wyglądają elementy $\Omega$. Mogły to być pary liczb, słonie, czy słowa TAK i NIE. Nie miało to dla nas większego znaczenia. Wyobraźmy sobie jednak, że chcemy obliczyć średnią sumę oczek z dwóch kostek albo wykonać histogram wagi słonia. Jak zinterpretować te zadania w obrębie naszej teorii?

Zarówno suma oczek na kostkach, jak i waga słonia, są pewnymi funkcjami, określonymi na odpowiednim $\Omega$ i przypisującymi każdemu zdarzeniu elementarnemu pewną interesującą nas cechę, która jest liczbą rzeczywistą. W pierwszym przypadku każdemu wynikowi rzutu dwiem kostkami przypisujemy sumę oczek, w drugim każdemu słoniowi jego wagę. Obliczanie średniej i wykonywanie histogramu będzie się odbywać na wartościach tak zdefiniowanej funkcji.

### Definicja zmiennej losowej (prawie)

**Uwaga 4.1**  

Definicje i rozważania w tym paragrafie nie są do końca poprawne, ale za to są zupełnie elementarne. Poprawiamy je w następnym paragrafie poświęconym teorii miary.

**Prawie definicja (Zmienna losowa)**  

Niech $(\Omega,P)$ będzie przestrzenią probabilistyczną. Zmienną losową (określoną na $\Omega$) nazywamy dowolną funkcję $X:\Omega \rightarrow \mathbb{R}$.

Jak widać, wbrew swojej nazwie, zmienne losowe nie są wcale zmiennymi, nie są też w żadnym sensie losowe.  

**Przykład 4.2**  

Rzut dwiema kostkami można w naturalny sposób opisać przestrzenią probabilistyczną $( \Omega, P)$, gdzie $\Omega = \{ (i,j) : 1 \le i,j \le 6 \}$, a $P$ jest zadane schematem klasycznym. Na takiej przestrzeni można określić zmienną losową $X$ opisującą sumę oczek następująco:  

$X( (i,j) ) = i+j$.  

Podobnie możemy postąpić dla iloczynu oczek, oczek na pierwszej kostce, itp.

Zmiennej losowej można użyć do definiowania zdarzeń w jej dziedzinie. Na przykład, za pomocą opisanej powyżej zmiennej $X$ możemy zdefiniować zdarzenia:

- $X = 5$ - suma oczek wynosi 5,
- $X \le 4$ - suma oczek jest nie większa niż 4,
- $X \in \{3,5,\ldots,11\}$ - suma oczek jest nieparzysta, itd.

Formalnie, napis $X \in A$ będziemy interpretować jako zdarzenie $\{\omega \in \Omega : X(\omega) \in A\} = X^{-1}(A)$, i analogicznie napisy $X = a$, $X \le a$, itp.  

Można pójść z tą notacją jeszcze dalej, np.  $sin(X) > X/20$. Dowolny napis postaci $\phi(X)$, gdzie $\phi(x)$ jest formułą logiczną zmiennej $X$ możemy zinterpretować jako $\{ \omega \in \Omega : \phi(X(\omega))\} = (X \circ \phi)^{-1}(prawda)$.  

Nic też nie stoi na przeszkodzie, aby definiować zdarzenia za pomocą więcej niż jednej zmiennej. Jeśli na przykład $X$ jest, jak poprzednio, sumą oczek z dwóch kostek, a $Y$ iloczynem, to możemy zdefiniować zdarzenie "suma oczek jest większa niż iloczyn" napisem $X > Y$. Tak jak poprzednio, dla dowolnej formuły logicznej $\phi(x,y)$ i zmiennych $X,Y$ definiujemy zdarzenie $\phi(X,Y)$ jako $\{\omega \in \Omega : \phi(X(\omega),Y(\omega))\}$.  

W naturalny sposób możemy interpretować formuły trzech i więcej zmiennych.

Jeśli interesują nas własności zmiennej losowej $X$ takie jak jej średnia wartość, tak czy inaczej zdefiniowany rozrzut wartości itp., to dziedzina $X$ oraz to jak konkretnie $X$ jest zdefiniowana nie mają dla nas znaczenia. Cała istotna informacja o zmiennej $X$ jest zawarta w tzw. rozkładzie $X$.

**Prawie definicja (Rozkład zmiennej losowej)**  

Rozkładem zmiennej losowej $X$ nazywamy przestrzeń probabilistyczną $(\mathbb{R},P_X)$, gdzie  

$P_X(A) = P(X \in A) = P(X^{-1}(A))$.

Będziemy też pisać $X \sim Y$, jeśli $X$ i $Y$ mają taki sam rozkład.

### Odrobina teorii miary

Nasze dotychczasowe rozważania zawierają niestety drobną lukę logiczną.  Otóż w definicji rozkładu $X$ pojawia się prawdopodobieństwo $P(X^{-1}(A))$, ale nie jest wcale powiedziane, że wartość ta jest określona. Zgodnie z definicją, przestrzeń probabilistyczna jest trójką $(\Omega, \mathcal{F},P)$. Umówiliśmy się pomijać $\sigma$-ciało $\mathcal{F}$ wszędzie tam, gdzie jest ono równe $2^\Omega$ lub po prostu nie ma znaczenia. W przypadku definicji zmiennej losowej $\mathcal{F}$ ma znaczenie - musimy zagwarantować, że $X^{-1}(A) \in \mathcal{F}$ dla tych zbiorów $A$, na których ma być określony rozkład $X$.

Przyjmiemy, że rozkład ma być określony na borelowskich podzbiorach $\mathbb{R}$, co prowadzi do następującej definicji zmiennej losowej:  

**Definicja (Zmienna losowa)**  

Niech $(\Omega,\mathcal{F},P)$ będzie przestrzenią probabilistyczną. Zmienną losową (określoną na $\Omega$) nazywamy dowolną funkcję $X:\Omega \rightarrow \mathbb{R}$ taką, że dla dowolnego zbioru borelowskiego $A \subseteq \mathbb{R}$ zachodzi $X^{-1}(A) \in \mathcal{F}$ (czyli $f$ musi być *mierzalna* względem $\mathcal{F}$ ).

Z kolei rozkład zmiennej losowej definiujemy następująco (uzupełniamy poprzednią definicję wskazując, które zbiory będą mierzalne):  

**Definicja (Rozkład zmiennej losowej)**  

Rozkładem zmiennej losowej $X$ nazywa przestrzeń probabilistyczną $\mathbb{R},\mathcal{B},P_X$, gdzie $\mathcal{B}$ jest rodziną borelowskich podzbiorów $\mathbb{R}$ oraz  

$P_X(A) = P(X \in A) = P(X^{-1}(A))$.

Założenie mierzalności funkcji będzie się pojawiać w tym i dalszych wykładach jeszcze kilkukrotnie. Za każdym razem jest to motywowane rozważaniami, które właśnie przeprowadziliśmy.

---

## Podstawowe rozkłady dyskretne

Omówimy teraz kilka najbardziej typowych rozkładów zmiennych losowych, przy czym przez najbliższych kilka wykładów ograniczymy się do tzw. rozkładów dyskretnych.  

**Definicja (Rozkład dyskretny)**  

Zmienna $X$ ma *rozkład dyskretny* (lub krócej: *jest dyskretna*) jeśli zachodzi  

$\sum_{x \in \mathbb{R}} P(X=x) = 1.$  

Innymi słowy, z prawdopodobieństwem $1$ zmienna $X$ przyjmuje jedną z przeliczalnie wielu wartości.

**Uwaga 4.3**  

W powyższej definicji pojawia się suma po zbiorze nieprzeliczalnym, co może budzić pewne zaniepokojenie. Proszę jednak zwrócić uwagę, że tylko przeliczalnie wiele wartości w tej sumie może być niezerowych na mocy przeliczalnej addytywności prawdopodobieństwa. W dalszej części wykładu będziemy dość często używać tego rodzaju notacji.

**Przykład 4.4**  

Łatwo podać przykład zmiennej dyskretnej - na przykład liczba rzutów do wypadnięcia pierwszego orła. Przykładem zmiennej niedyskretnej jest czas $T$ oczekiwania w kolejce. O ile nie jest to bardzo nietypowa kolejka, to dla każdej liczby $t > 0$ zachodzi $P(T=t) = 0$ (może się zdarzyć, że $P(T=0) > 0$). A zatem $\sum_{t \in \mathbb{R}} P(T=t) = P(T=0)$ i z reguły jest to wartość mniejsza niż $1$. Zmiennymi tego rodzaju zajmiemy się za kilka wykładów.

W tym wykładzie interesować nas będą wyłącznie zmienne dyskretne. Co więcej, z reguły będą one przyjmować jedynie wartości naturalne.

Poniższy fakt opisuje bardzo przydatną własność zmiennych dyskretnych, często uznaje się go wręcz za ich definicję:  

**Fakt 4.5**  

Jeśli $X$ jest zmienną dyskretną, a $A \subseteq \mathbb{R}$, to  

$P(X \in A) = \sum_{x \in A} P(X=x).$  

**Dowód**  

Zachodzi oczywiście $P(X \in A) \ge \sum_{x \in A} P(X=x)$ na mocy przeliczalnej addytywności prawdopodobieństwa. Podobnie $P(X \not\in A) \ge \sum_{x \not\in A} P(X=x)$.  Dodając stronami dostajemy $1 \ge 1$, a równość jest możliwa tylko jeśli zachodzi teza.

Fakt 4.5 pokazuje, że cała informacja o zmiennej dyskretnej $X$ jest zawarta w wartościach $P(X=x)$. Często wprowadza się oznaczenie $f_X(x) = P(X=x)$, a funkcję $f_X$ nazywa się *funkcją prawdopodobieństwa $X$*. Nie będziemy tego jednak robić w tym wykładzie.

Przyjrzymy się teraz kilku najważniejszym rozkładom dyskretnym.

### Rozkład Bernoulliego

**Definicja (Rozkład Bernoulliego)**  

Zmienna $X$ ma *rozkład Bernoulliego* jeśli przyjmuje tylko dwie wartości: 0 i 1. Prawdopodobieństwa tych wartości oznacza się zwyczajowo: $p = P(X=1)$ oraz $q = P(X=0)$. Wartość 1 nazywa się często *sukcesem*, a wartość 0 *porażką*.

Zmienne takie pojawiają się w naturalny sposób, jeśli przeprowadzamy pewne doświadczenie i interesuje na to, czy zakończyło się ono w konkretny sposób, np. w rzucie monetą wypadł orzeł bądź nie, pacjent przeżył operację bądź nie, itp. Jak wkrótce zobaczymy, zmienne o rozkładzie Bernoulliego pełnią często rolę "klocków", z których składa się bardziej skomplikowane zmienne.

### Rozkład jednostajny

**Definicja (Rozkład jednostajny (dyskretny))**  

Zmienna $X$ ma *dyskretny rozkład jednostajny na przedziale $[a,b]$*, gdzie $a,b \in \mathbb{Z}$ , jeśli przyjmuje wartości $a,a+1,\ldots,b$, przy czym wszystkie one są równie prawdopodobne, t.j. $P(X=a) = \ldots = P(X=b) = \frac{1}{b-a+1}$.

Rozkład jednostajny ma na przykład liczba oczek na kostce, czy wynik losowania liczby naturalnej z zadanego przedziału (ten drugi przykład jest dość marny, jako że jest właściwie równoważny definicji).

### Rozkład dwumianowy

**Przykład 4.6**  

Rzucamy $n$ razy monetą, na której orzeł wypada z prawdopodobieństwem $p$. Niech $X$ będzie liczbą orłów. Jaki rozkład ma $X$?

Mamy oczywiście $P(X=k) = 0$ dla $k \not \in \{0,\ldots,n\}$. Jeśli $k \in \{0,\ldots,n\}$, to mamy ${n \choose k}$ różnych sposobów na uzyskanie $k$ orłów. Korzystając z niezależności zdarzeń odpowiadających wynikom różnych rzutów łatwo pokazać, że prawdopodobieństwo dla każdego z tych sposobów jest równe $p^k(1-p)^{n-k}$. A zatem $P(X=k) = {n \choose k}p^k(1-p)^{n-k}$.

**Definicja (Rozkład dwumianowy)**  

Zmienna $X$ ma *rozkład dwumianowy z parametrami $n,p$*, ozn. $X \sim Binom(n,p)$, jeśli $P(X=k) = {n \choose k}p^k(1-p)^{n-k}$ dla $k=0,\ldots,n$ i $P(X=k)=0$ dla pozostałych $k$.

Ciąg niezależnym doświadczeń (tzw. prób), z których każda kończy się sukcesem z tym samym prawdopodobieństwem, nazywamy *ciągiem prób Bernoulliego* (wynik każdej z prób jest zmienną o rozkładzie Bernoulliego!). Zmienna o rozkładzie dwumianowym opisuje liczbę sukcesów w ciągu prób Bernoulliego.

**Uwaga 4.7**  

Notację $X \sim Binom(n,p)$ można traktować jako sposób zapisania faktu, że zmienna $X$ ma rozkład dwumianowy. Można też przyjąć, że $Binom(n,p)$ jest pewną wzorcową zmienną o rozkładzie dwumianowym i wtedy napis $X \sim Binom(n,p)$ oznacza, że obie zmienne mają ten sam (dwumianowy) rozkład.

### Rozkład geometryczny

**Przykład 4.8**  

Rzucamy monetą, na której orzeł wypada z prawdopodobieństwem $p$ aż do uzyskania pierwszego orła. Niech $X$ będzie liczbą rzutów (łącznie z rzutem, w którym wypadł orzeł). Jaki rozkład ma $X$?

Oczywiście $P(X=k) > 0$ tylko dla $k \in \mathbb{N}\setminus \{0\}$. Mamy $P(X=1) = p$. Korzystając z niezależności dostajemy $P(X=2) = (1-p)p$ i ogólnie $P(X=k) = (1-p)^{k-1}p$.

**Definicja (Rozkład geometryczny)**  

Zmienna $X$ ma *rozkład geometryczny z parametrem $p$*, ozn. $X \sim Geom(p)$, jeśli $P(X=k) = (1-p)^{k-1}p$ dla $k \in \mathbb{N}\{0\}$ i $P(X=k)=0$ dla pozostałych $k$.

Zmienna o rozkładzie geometrycznym opisuje numer pierwszej próby zakończonej sukcesem w ciągu prób Bernoulliego.

### Rozkład Poissona

**Przykład 4.9**  

Kierownik laboratorium komputerowego otrzymuje średnio $\lambda$ informacji o awarii komputera na miesiąc. Niech $X$ będzie liczbą takich informacji w konkretnym miesiącu. Jaki rozkład ma $X$?

Podzielmy miesiąc na $n$ przedziałów czasowych. Załóżmy, że $n$ jest na tyle duże, że prawdopodobieństwo otrzymania dwóch informacji o awarii w jednym przedziale jest zaniedbywalne. Załóżmy ponadto, że zdarzenia odpowiadające otrzymaniu informacji o awarii w różnych przedziałach są niezależne. Wtedy mamy do czynienia z ciągiem $n$ prób Bernoulliego. Jakie jest prawdopodobieństwo sukcesu (niezbyt satysfakcjonującego, ale trzymajmy się konwencji)? Skoro spośród $n$ przedziałów średnio $\lambda$ powinno zawierać awarię, to intuicyjnie prawdopodobieństwo sukcesu $p$ powinno być równe $\frac{\lambda}{n}$. Wydaje się więc, że rozkład naszej zmiennej $X$ jest dobrze przybliżany przez $Binom(n,\frac{\lambda}{n})$ dla dużych $n$, t.j.  

$P(X=k) \sim {n \choose k} (\frac{\lambda}{n})^k (1-\frac{\lambda}{n})^{n-k}$.  

Można pokazać (ćwiczenia), że dla $n \rightarrow \infty$ wyrażenie to zbiega do  $\frac{e^{-\lambda}\lambda^k}{k!}$.  

Powyższe rozwiązanie ma oczywiście w jakimś stopniu charakter heurystyczny. W szczególności nie jest wcale jasne, czy nasze założenia są spełnione. Może być na przykład tak, że awarie komputerów mają tendencję do występowania razem (mogą być na przykład powodowane awariami zasilania). Wtedy poszczególne próby nie byłyby niezależne. Okazuje się jednak, że otrzymany przez nas wzór bardzo dobrze opisuje wiele sytuacji praktycznych.

**Definicja (Rozkład Poissona)**  

Zmienna $X$ ma *rozkład Poissona z parametrem $\lambda$*, ozn. $X \sim Pois(\lambda)$, jeśli $P(X=k) = \frac{e^{-\lambda}\lambda^k}{k!}$ dla $k \in \mathbb{N}$ i $P(X=k)=0$ dla pozostałych $k$.

Rozkład Poissona dobrze opisuje procesy podobne do sytuacji opisanej w przykładzie 4.9: liczbę telefonów odebranych w centrum obsługi klienta przez godzinę, liczbę błędów ortograficznych na $1000$ znaków rękopisu, itp.

Istnieje jeszcze inne, nieco zaskakujące, zastosowanie rozkładu Poissona. Zwróćmy uwagę, że rozkład Poissona uzyskaliśmy jako granicę rozkładów dwumianowych z dużym $n$, małym $p$ i $np = \lambda$. W związku z tym można użyć rozkładu Poissona $Pois(np)$ do przybliżenia rozkładu dwumianowego $Binom(n,p)$ o ile tylko $n$ jest duże ($n$ powinno być dużo większe niż interesująca nas liczba sukcesów $k$), a $p$ małe. Co ciekawe, prowadzi to z reguły do prostszych rachunków.

**Przykład 4.10**  

Gramy $n=1000000$ razy w loterii, w której prawdopodobieństwo wygrania wynosi $p=\frac{1}{500000}$. Jakie jest prawdopodobieństwo tego, że wygramy dokładnie $k=2$ razy?

Liczba wygranych ma rozkład $Binom(n,p)$, a zatem szukane prawdopodobieństwo jest równe  

${n \choose 2}p^2 (1-p)^{n-2}.$  

Trzeba to jeszcze tylko obliczyć...

Możemy jednak uniknąć rachunków zauważając, że dla małych $k$ (a nasze $k$ jest bardzo małe) możemy uzyskać dobre przybliżenie odpowiedzi korzystając z rozkładu $Pois(np)$ . Tym razem dostajemy dużo prostszy wzór  

$\frac{e^{-2} 2^2}{2} = \frac{2}{e^2} \sim 0.27.$

---

## Dalsze definicje i własności

### Rozkład warunkowy

Ponieważ napisy $X \in A$ czy $X \ge Y$ interpretujemy jako zdarzenia, możemy bez żadnych dodatkowych definicji używać pojęcia prawdopodobieństwa warunkowego na przykład tak:  

$P(X \in A | B)$  

lub tak:  

$P (X \ge Y | Y \ge 1)$.

Warto w tym miejscu zwrócić szczególną uwagę na pierwsze z wyrażeń. Przypomnijmy, że para $(\Omega,P(\cdot|B))$ jest przestrzenią probabilistyczną. Jeśli potraktujemy $X$ jako zmienną losową określoną na tej przestrzeni, to otrzymamy pewną nową zmienną, z reguły oznaczaną $X|B$, dla której  

$P( X|B \in A) = P(X \in A | B)$.  

Jest to często używany skrót notacyjny. Rozkład zmiennej $X|B$ nazywa się *rozkładem $X$ pod warunkiem $B$*.

**Uwaga 4.11**  

Nie należy mylić wprowadzonej tu notacji ze spotykaną w literaturze notacją $X|Y$, gdzie $X,Y$ są zmiennymi losowymi. Tej drugiej nie będziemy definiować w ramach tego wykładu.

### Niezależność zmiennych losowych

Jak zdefiniować niezależność zmiennych losowych. Następująca intuicyjna definicja działa bardzo dobrze  

**Definicja (Niezależność zmiennych losowych)**  

Zmienne losowe $X,Y$ są niezależne jeśli dla każdych zbiorów borelowskich $A,B \subseteq \mathbb{R}$ zachodzi  

$P(X \in A \wedge Y \in B) = P(X \in A) P(Y \in B)$.

**Uwaga 4.12**  

Zamiast niezależności zdarzeń $X \in A$ i $Y \in B$ dla dowolnych zbiorów borelowskich, można zażądać niezależności dla $A,B$ przedziałów, lub po prostu niezależności zdarzeń $X \le x$ i $Y \le y$ dla dowolnych $x,y \in \mathbb{R}$. Równoważność tych definicji wynika z definicji zbiorów borelowskich, ale jej dowód nie jest prosty.

Co ciekawe, i bardzo przydatne w obliczeniach, dla dyskretnych zmiennych losowych istnieje prostsza charakteryzacja niezależności  

**Fakt 4.13**  

Dyskretne zmienne losowe $X,Y$ są niezależne wtedy i tylko wtedy, gdy dla każdych $x,y \in \mathbb{R}$ zachodzi  

$P(X=x \wedge Y = y) = P(X =x) P(Y = y)$.  

**Dowód**  

Jeśli dla dowolnych $x,y \in \mathbb{R}$ zachodzi powyższa równość, to $X,Y$ są niezależne, bo  

$P(X \in A \wedge Y \in B) = \sum_{x \in A \wedge y \in B} P(X = x \wedge Y = y) =  \sum_{x \in A \wedge y \in B} P(X = x)P(Y = y) = \sum_{x \in A} P(X = x)\sum_ {y \in B}P(Y = y)  = P(X \in A) P(Y \in B)$.

Implikacja w drugą stronę jest oczywista.

### Złożenia

Składając zmienną losową $X$ z funkcją $f:\mathbb{R}\rightarrow\mathbb{R}$ otrzymujemy nową zmienną losową oznaczaną $f(X)$. Oczywiście w ogólnym przypadku musimy założyć, że funkcja $f$ jest mierzalna.  

**Przykład 4.14**  

Jeśli $X$ jest liczbą oczek w rzucie kostką, to:

- dla $f(x) = x^2$ dostajemy zmienną $f(X)$, która jest kwadratem liczby oczek,
- dla $f(x) = x \mod 2$ dostajemy zmienną 0/1-kową $f(X)$, która jest równa $1$, gdy wypada nieparzysta liczba oczek.

Oczywiście zamiast pisać $f(X)$ i definiować osobno funkcję $f$ możemy pisać po prostu $X^2$ czy $X \mod 2$.

Aby upewnić się, czy dobrze rozumiemy jak składać zmienne losowe z funkcjami, sprawdźmy jak wygląda rozkład zmiennej $f(X)$:  

$P(f(X) \in A) = P( X \in f^{-1}(A) )$.

Nie ma powodu, dla którego mielibyśmy się ograniczać do jednej zmiennej i funkcji jednoargumentowych. Jeśli na przykład $X,Y$ są zmiennymi losowymi, a $f:\mathbb{R}^2 \rightarrow \mathbb{R}$ jest funkcją mierzalną, to możemy zdefiniować nową zmienną losową $f(X,Y)$. Analogicznie postępujemy w przypadku większej liczby zmiennych.

W ten sposób, jeśli $X,Y$ są zmiennymi losowymi, to zmienną losową jest też ich suma $X+Y$, iloczyn $XY$, czy nawet $X^Y$.

Jak wygląda rozkład $f(X,Y)$?  

$P(f(X,Y) \in A) = P( (X,Y) \in f^{-1}(A) ).$  

Ten wzór pokazuje, że do obliczenia rozkładu zmiennej $f(X,Y)$ nie wystarczy znajomość rozkładów zmiennych $X$ i $Y$. Musimy znać ich tzw. *rozkład łączny* tzn. prawdopodobieństwa postaci $P(X \in A \wedge Y \in B)$.

Sytuacja jest wyjątkowo prosta jeśli zmienne $X$ i $Y$ są niezależne. Mamy wtedy:  

$P(f(X,Y) \in A) = P( (X,Y) \in f^{-1}(A) ) = \sum_{(x,y) \in f^{-1}(A)} P(X=x)P(Y=y).$

W szczególności zachodzi następujący bardzo przydatny fakt:  

**Fakt 4.15**  

Jeśli $X,Y$ są niezależnymi zmiennymi dyskretnymi, to  

$P(X+Y=k) = \sum_{x \in \mathbb{R}} P(X=x) P(Y=k-x)$.

Dowód wynika natychmiast z naszych dotychczasowych rozważań.
