---
layout: "default"
title: "Wykład 7: Ciągłe zmienne losowe"
source_url: "http://smurf.mimuw.edu.pl/node/712"
source_kind: "html"
uses_math: true
---

# Wykład 7: Ciągłe zmienne losowe

## Motywacja  i definicja

Dotychczas koncentrowaliśmy uwagę na zmiennych dyskretnych, t.j. takich, że zachodzi $\sum_{x \in \mathbb{R}} P(X=x) = 1$. Innymi słowy, istnieje pewien przeliczalny zbiór wartości o niezerowym prawdopodobieństwie, i z prawdopodobieństwem $1$ zmienna przyjmuje jedną z tych wartości.

Łatwo sobie jednak wyobrazić sytuacje, w których pojawiają się zmienne losowe nie mające tej własności. Jeśli na przykład losujemy punkt z odcinka $[0,1]$ i $X$ jest wylosowanym punktem, to musi zachodzić $P(X=x) = 0$ dla każdego $x \in [0,1]$. Gdyby bowiem było $P(X=x_0) = p>0$ dla pewnego $x_0$, to musiałoby też być $P(X=x) = p$ dla każdego $x$ (dlaczego $x_0$ miałby być bardziej prawdopodobny?), co prowadzi do sprzeczności z $P(X \in [0,1]) = 1$.

Podobnie, jeśli mierzymy (dokładnie, wynikiem może być dowolna liczba rzeczywista) prędkość samochodu na autostradzie, czy wzrost losowo wybranej osoby, to sensownie jest założyć, że prawdopodobieństwo każdego konkretnego wyniku jest równe $0$.

Jest to dość niepokojąca sytuacja - cała teoria jaką dotychczas omówiliśmy opierała się dość mocno na założeniu dyskretności. W szczególności, własnością zmiennych dyskretnych, która była kluczowa w wielu definicjach i dowodach było to, że dla dowolnego zbioru $A\subseteq\mathbb{R}$ i zmiennej dyskretnej $X$ zachodzi $P(X \in A) = \sum_{x \in A} P(X=x)$.

Okazuje się, że w obu sytuacjach opisanych powyżej, a także w wielu innych, można opisać $P(X \in A)$ w sposób tylko trochę bardziej skomplikowany.  

**Definicja (Rozkład ciągły)**  

Zmienna $X$ ma rozkład ciągły, jeśli istnieje funkcja $f_X:\mathbb{R} \rightarrow \mathbb{R}_{\ge 0}$ taka, że dla każdego przedziału $[a,b]$ zachodzi $P(X \in [a,b]) = \int_a^b f_X(x) dx$ (lub równoważnie: dla każdego zbioru mierzalnego $A \subseteq \mathbb{R}$ zachodzi $P(X \in A) = \int_A f_X(x) dx$).  

Funkcję $f_X$ nazywamy gęstością zmiennej $X$.

**Uwaga 7.1**  

Łatwo zauważyć, że jeśli $f$ jest gęstością pewnej zmiennej, to $\int_{-\infty}^\infty f(x) dx = 1$. Z drugiej strony, jeśli pewna funkcja $f:\mathbb{R} \rightarrow \mathbb{R}_{\ge 0}$ spełnia ten warunek, to jest gęstością pewnej zmiennej. Dlatego w dalszej części wykładu, definiując rozkład za pomocą funkcji gęstości, będziemy zmuszeni zawsze sprawdzić, czy opisywana przez nas funkcja faktycznie może być gęstością, t.j. czy zachodzi $\int_{-\infty}^\infty f(x) dx = 1$.

**Uwaga 7.2**  

Zmienną losową o rozkładzie ciągłym można opisać za pomocą więcej niż jednej funkcji gęstości. Jeśli bowiem dowolną funkcję gęstości zmodyfikujemy na zbiorze miary $0$, to otrzymana funkcja też będzie dobrą funkcją gęstości. Z powodu tej niejednoznaczności w sformułowaniach niektórych twierdzeń tego wykładu pojawiają się słowa "prawie wszędzie".

Jaki jest sens funkcji gęstości? Załóżmy, że $I$ jest odcinkiem na tyle małym, że $f_X$ jest na nim niemal stała (znalezienie takiego przedziału może czasem być niemożliwe, ale nie będziemy się tym przejmować, szukamy tylko intuicji). Wtedy  

$P(X \in I) = \int_I f_X(x) dx \approx |I| f_X(x)$,  

dla dowolnego $x \in I$, czyli  

$f_X(x) \approx \frac{P(X \in I)}{|I|}$.  

Innymi słowy, $f_X(x)$ mówi nam jak dużo prawdopodobieństwa przypada na przedział długości $1$ w okolicy punktu $x$, czyli jest taką "lokalną gęstością prawdopodobieństwa" w okolicy punktu $x$, co tłumaczy nazwę.

**Przykład 7.3**  

Mogłoby się wydawać, że każda zmienna losowa musi być albo dyskretna albo ciągła, ale łatwo zauważyć, że tak być nie musi. Wyobraźmy sobie, że chcemy wymodelować czas oczekiwania w kolejce u fryzjera (ew. w sklepie itp.) za pomocą zmiennej losowej $X$. Z niezerowym prawdopodobieństwem nie będziemy w ogóle czekać, a więc $P(X=0) > 0$. Jeśli jednak $X>0$ to wydaje się, że mamy do czynienia z rozkładem ciągłym, w szczególności żadna wartość $X$ większa niż $0$ nie będzie mieć niezerowego prawdopodobieństwa. Ta zmienna nie jest ani ciągła, ani dyskretna, jest jednak pewnego rodzaju kombinacją zmiennej ciągłej i dyskretnej. Część prawdopodobieństwa jest skoncentrowana w punkcie $0$, resztę można opisać funkcją gęstości.  

Czy każda zmienna losowa ma taką postać? Na to pytanie odpowiemy w dalszej części tego wykładu.

---

## Dystrybuanta zmiennej losowej

Naturalnym pojęciem związanym z pojęciem zmiennej losowej jest dystrybuanta.  

**Definicja (dystrybuanta)**  

Dystrybuantą zmiennej losowej $X$ jest funkcja $F_X:\mathbb{R}\rightarrow\mathbb{R}$ określona  $F_X(x) = P(X \le x)$.

Zacznijmy od sformułowania kilku prostych własności dystrybuanty:  

**Fakt 7.4 (własności dystrybuanty)**  

Niech $X$ będzie zmienną losową, a $F_X$ jej dystrybuantą. Wtedy

1. $F_X$ jest niemalejąca,
2. $F_X$ jest prawostronnie ciągła,
3. $lim_{x \rightarrow -\infty} F_X(x) = 0$ oraz $lim_{x \rightarrow \infty} F_X(x) = 1$.

Dowody wszystkich własności są oczywiste. Można też pokazać (dowód jest dość techniczny), że własności te charakteryzują funkcje, które są dystrybuantami:  

**Twierdzenie 7.5**  

Jeśli $F_X:\mathbb{R}\rightarrow\mathbb{R}$ spełnia warunki z faktu 7.4, to istnieje zmienna losowa $X$, dla której $F_X$ jest dystrybuantą.

Następne twierdzenie i wniosek z niego pokazują, że patrząc na dystrybuantę można odróżnić zmienne dyskretne od ciągłych (oczywiste dowody pomijamy).

**Twierdzenie 7.6**  

Jeśli $X$ jest zmienną losową, a $F_X$ jej dystrybuantą, to dla każdego $x\in \mathbb{R}$ zachodzi  

$P(X=x) = F_X(x) - F_X(x^-)$,  

gdzie  

$F_X(x^-) = \lim_{y \rightarrow x, y < x} F_X(y)$  

jest lewostronną granicą $F_X$ w $x$.

**Wniosek 7.7**  

Jeśli $X$ jest zmienną ciągłą, to $F_X$ jest funkcją ciągłą. Jeśli natomiast $X$ jest dyskretna to  

$\sum_{x \in \mathbb{R}} (F_X(x) - F_X(x^-)) = 1$.

**Uwaga 7.8**  

Chciałoby się powiedzieć, że gdy $X$ jest dyskretna, to $F_X$ jest schodkowa, i z reguły tak właśnie jest - w szczególności jest to prawdą dla zmiennych o rozkładach, które poznaliśmy w dotychczasowych wykładach. Istnieją jednak zmienne dyskretne, których dystrybuanty nie są stałe na żadnym przedziale. Wystarczy w tym celu przypisać niezerowe prawdopodobieństwa elementom przeliczalnego zbioru gęstego w $\mathbb{R}$, np. wszystkim liczbom wymiernym.

Następne twierdzenie podaje ważną charakteryzację zmiennych ciągłych za pomocą ich dystrybuant (dowód pomijamy)  

**Twierdzenie 7.9**  

Jeśli $X$ jest zmienną ciągłą, to:

- $F_X$ jest różniczkowalna prawie wszędzie,
- $F_X'(t) = f_X(t)$ (prawie wszędzie).

Z drugiej strony, jeśli $F$ jest dystrybuantą różniczkowalną prawie wszędzie, a $f = F'$ (tam gdzie $F$ nie jest różniczkowalna, $f$ przyjmuje dowolną wartość, np. $0$), to $f$ jest gęstością ciągłej zmiennej losowej, o ile tylko $\int_{-\infty}^{\infty} f(x) dx = 1$.

To, że ten ostatni warunek jest konieczny pokazuje poniższy przykład. Przy okazji odpowiadamy na pytanie postawione w przykładzie 7.3.

**Przykład 7.10 (Funkcja Cantora, czyli diabelskie schody)**  

Pokażemy zmienną $X$ taką, że:

- $P(X=x) = 0$ dla każdego $x \in \mathbb{R}$ (czyli $X$ nie ma części dyskretnej), ale też
- na żadnym przedziale $[a,b]$ dla którego $P(X \in [a,b]) > 0$, nie da się zdefiniować funkcji $f:[a,b] \rightarrow \mathbb{R}$ takiej, że $P(X \in I) = \int_I f(t) dt$ dla każdego przedziału $I \subseteq [a,b]$ (a więc $X$ nie ma gęstości na żadnym przedziale o niezerowym prawdopodobieństwie, czyli nie ma części ciągłej)

Zmienną X zdefiniujemy za pomocą jej dystrybuanty. Rozważmy następujący ciąg funkcji $F_i$:

- $F_0(x) = 0$ dla $x < 0$ i $F_0(x) = 1$ dla $x > 1$, oraz $F_0(x) = x$ dla $x \in [0,1]$.
- $F_{n+1}$ powstaje z $F_n$ w następujący sposób: Niech $I=[a,b]$ będzie dowolnym maksymalnym przedziałem na którym $F_n$ jest ściśle rosnąca. Dzielimy $I$ na 3 równej długości podprzedziały $I_1=[a,a+\frac{b-a}{3}]$, $I_2=[a+\frac{b-a}{3},a+2\frac{b-a}{3}]$, $I_3=[a+2\frac{b-a}{3},b]$. Definiujemy $F_{n+1} (x) = \frac{F_n(a)+F_n(b)}{2}$ dla $x \in I_2$, natomiast na przedziale $I_1$ funkcja $F_{n+1}$ rośnie liniowo od $F_n(a)$ do $\frac{F_n(a)+F_n(b)}{2}$ i odpowiednio na $I_3$ od $\frac{F_n(a)+F_n(b)}{2}$ do $F_n(b)$. W ten sposób postępujemy dla wszystkich maksymalnych przedziałow $I$, na których $F_n$ jest ściśle rosnąca.

Łatwo sprawdzić, że ciąg funkcji $F_n$ jest zbieżny punktowo do pewnej funkcji. Niech $F_X=F$ będzie granicą $F_n$ i niech $X$ będzie zmienną losową o dystrybuancie $F$. Łatwo pokazać, że $F$ jest ciągła, a zatem dla każdego $x$ zachodzi $P(X=x) = 0$ na mocy twierdzenia 7.6. Można też pokazać (co jest nieco trudniejsze), że $F_X$ ma zerową pochodną wszędzie poza zbiorem miary zero (jest to tzw. zbiór Cantora), a zatem nie ma gęstości na żadnym przedziale $[a,b]$ dla którego $P([a,b]) > 0$.

---

## Przykłady rozkładów ciągłych

### Rozkład jednostajny

**Definicja (rozkład jednostajny)**  

Zmienna $X$ o rozkładzie jednostajnym na przedziale $[a,b]$ dla $a < b$, ozn. $X \sim Unif(a,b)$ ma gęstość $f_X$, gdzie $f_X(x) = \frac{1}{b-a}$ dla $x \in [a,b]$ i $f_X(x) = 0$ dla pozostałych $x$.

Rozkład $Unif(a,b)$ pojawia się, gdy losujemy liczbę z przedziału $[a,b]$ tak, aby prawdopodobieństwo uzyskania wyniku w dowolnym przedziale $I$ było proporcjonalne do długości tego przedziału. Intuicyjnie chcemy, żeby wszystkie liczby były "równie prawdopodobne", choć oczywiście w przypadku losowania z przedziału sformułowanie "równie prawdopodobne" nie ma zbyt wiele sensu (wszystkie wyniki oczywiście są równie prawdopodobne, bo wszystkie mają prawdopodobieństwo $0$, ale przecież nie o to nam chodzi).

### Rozkład  wykładniczy

**Definicja (rozkład wykładniczy)**  

Zmienna $X$ o rozkładzie wykładniczym z parametrem $\theta > 0$, ozn. $X \sim Exp(\theta)$ ma gęstość $f_X$, gdzie $f_X(x) = \theta e^{-\theta x}$ dla $x \ge 0$ i $f_X(x) = 0$ dla $x < 0$.

Ten rozkład dobrze modeluje czas oczekiwania na zdarzenie, które ma cały czas "taką samą szansę zajścia", na przykład czas do następnego telefonu w centrum telefonicznym, czas do zajścia rozpadu radioaktywnego, itp. Można go też używać do modelowania czasu życia organizmów lub wszelkiego rodzaju sprzętu, aczkolwiek rozkład wykładniczy nie modeluje tych czasów bardzo dobrze. W obu przypadkach śmierć/awaria jest nieco bardziej prawdopodobna na początku, jest też bardziej prawdopodobna po upływie wystarczająco długiego czasu.

Sprawdzimy teraz, że funkcja $f_X$ z definicji rozkładu wykładniczego rzeczywiście jest gęstością (t.j. ma całkę równą 1), a przy okazji znajdziemy dystrybuantę rozkładu wykładniczego. Dla dowolnego $a \ge 0$ mamy:  

$\int_{a}^{\infty} f_X(t) dt = \int_a^\infty \theta e^{-\theta t}dt = \int_a^\infty - (e^{-\theta t})' dt = (-e^{-\theta t})|_a^\infty = 0 - (-e^{-\theta a}) = e^{-\theta a}$.

Stąd  

$\int_{-\infty}^\infty f_X(t) dt = \int_0^\infty f_X(t) dt = e^0 = 1$,  

czyli $f_X$ jest gęstością.

Ponadto  

$F_X(a) = P(X < a) = 1-P(X \ge a) = 1-\int_a^\infty f_X(t) dt = 1-e^{-\theta a}.$

O rozkładzie wykładniczym można myśleć jako o "ciągłej wersji" rozkładu geometrycznego. W szczególności każdej wartości $\theta > 0$  odpowiada pewna wartość $p$ taka, że dystrybuanty rozkładów $Exp(\theta$) i $Geom(p)$ przyjmują te same wartości dla wszystkich argumentów naturalnych (ćwiczenia).

### Rozkład normalny

**Definicja (rozkład normalny lub Gaussa)**  

Zmienna $X$ o rozkładzie normalnym o wartości oczekiwanej $\mu$ i wariancji $\sigma^2$, ozn. $N(\mu,\sigma^2)$ ma gęstość  

$f_X(x) = \frac{1}{\sqrt{2\pi}\sigma} e^{-\frac{(x-\mu)^2}{2\sigma^2}}$.

Definicja rozkładu normalnego jest dość skomplikowana, jest on jednak niezwykle ważny. Jest ku temu kilka powodów, najważniejszym jest tzw. Centralne Twierdzenie Graniczne (które pojawi się pod koniec tego wykładu), które mówi, że suma dużej liczby niezależnych zmiennych, z których żadna nie dominuje pozostałych (t.j. nie przyjmuje dużo większych wartości, lub inaczej, nie ma decydującego wpływu na wynik) ma w przybliżeniu rozkład normalny. Wiele wielkości ma taki właśnie charakter - jest sumą wielu małych i niezależnych elementów - i co za tym idzie ma rozkład bliski normalnemu.  

Każdy na pewno nie raz widział charakterystyczny kształt dzwonu na histogramach ilustrujących różnego rodzaju statystyki.

Często zakłada się na przykład, że wzrost/masa człowieka, ew. wymiary/masa innych organizmów mają rozkład normalny. Należy tu oczywiście być ostrożnym: kobiety są generalnie niższe niż mężczyźni, można też zaobserwować różnice we wzroście pomiędzy poszczególnymi rasami. W związku z tym odpowiedni rozkład nie będzie miał kształtu dzwonu z jednym maksimum, ale raczej sumy kliku dzwonów. Łatwo zrozumieć dlaczego rozumowanie oparte na Centralnym Twierdzeniu Granicznym nie działa w tym przypadku: zarówno płeć jak i rasa są czynnikami, których wpływ na wzrost dominuje pozostałe czynniki. Jeśli jednak odpowiednio ograniczymy rozpatrywaną populację, np. do kobiet rasy białej, to rozkład wzrostu będzie bliski normalnemu.

Spróbujemy teraz sprawdzić, że funkcja $f_X$ z definicji rozkładu normalnego rzeczywiście jest gęstością. Zacznijmy od przypadku, w którym $\mu=0$ i $\sigma^2 = 1$, t.j. od rozkładu $N(0,1)$.

Chcemy obliczyć całkę $I = \int_{-\infty}^{\infty} f_X(x) dx$. Zamiast tego obliczymy jej kwadrat  

$I^2 = (\int_{-\infty}^{\infty} f_X(x) dx)(\int_{-\infty}^{\infty} f_X(y) dy) = \int_{-\infty}^{\infty}\int_{-\infty}^{\infty} f_X(x) f_X(y) dydx$.  

Mamy z definicji $f_X$  

$I^2 = \int_{-\infty}^{\infty}\int_{-\infty}^{\infty}  \frac{1}{2\pi} e^{-\frac{x^2+y^2}{2}} dydx$.

Korzystamy z tzw. podstawienia biegunowego t.j. $x = r\sin\theta$, $y = r\cos\theta$ i otrzymujemy  

$I^2 = \int_0^{2\pi} \int_0^\infty \frac{1}{2\pi} e^{-\frac{r^2}{2}} r dr d\theta$.  

Dodatkowe $r$ w tej całce jest modułem wyznacznika macierzy pochodnych cząstkowych $x$ i $y$ po $r$ i $\theta$ zgodnie z wielowymiarowym wzorem na całkowanie przez podstawienie. Łatwo zauważyć, że zewnętrzna całka jest równoważna mnożeniu przez $2\pi$, a zatem dostajemy  

$I^2 = \int_0^\infty e^{-\frac{r^2}{2}} r dr$.

Funkcja pod całką szczęśliwie (ale zgodnie z planem) jest pochodną funkcji $-e^{-\frac{r^2}{2}}$, a zatem  

$I^2 = (-e^{-\frac{r^2}{2}})|_0^\infty = 0 - (-1) = 1$,  

czyli $I=1$, co kończy obliczenia dla rozkładu $N(0,1)$.

Aby uzyskać analogiczny wynik w ogólnym przypadku, t.j. obliczyć całkę  

$J =  \int_{-\infty}^\infty \frac{1}{\sqrt{2\pi}\sigma} e^{-\frac{(x-\mu)^2}{2\sigma^2}}dx$  

wystarczy dokonać podstawienia $y=\frac{x-\mu}{\sigma}$ i okazuje się, że $J=I=1$.

**Uwaga 7.11**  

Rozkład $N(0,1)$, od którego rozpoczęliśmy nasze rozważania ma swoją nazwę - jest to tzw. *standardowy rozkład normalny*. Rozkład ten jak zobaczyliśmy, ma wyjątkowo prostą postać. Często pojawia się on w definicjach innych rozkładów, a także w rozumowaniach - jako najprostszy przypadek rozkładu normalnego. Rozkład ten ma też duże znaczenie historyczne z powodów, które w dzisiejszych czasach mogą nie być zupełnie oczywiste. Występuje on mianowicie w wielu rozumowaniach i procedurach wnioskowania statystycznego. Jednym z kroków takich procedur jest często odczytanie wartości dystrybuanty odpowiedniego rozkładu normalnego, ew. jej odwrotności, w pewnych punktach. W dzisiejszych czasach można te wartości w prosty sposób uzyskać za pomocą dowolnego pakietu statystycznego, kiedyś używano tablic matematycznych. Stablicowanie dystrybuant wszystkich rozkładów normalnych nie jest oczywiście możliwe, dlatego używano tylko tablic dla rozkładu standardowego, a metody wnioskowania formułowano tak, aby takie tablice wystarczały.

---

## Wartość oczekiwana i wariancja zmiennych o rozkładzie ciągłym

Wartość oczekiwaną dla zmiennych ciągłych definiujemy podobnie jak dla zmiennych dyskretnych  

**Definicja (wartość oczekiwana zmiennej ciągłej)**  

Niech $X$ będzie ciągłą zmienną losową o gęstości $f_X$. Wartością oczekiwaną $X$ nazywamy  

$EX = \int_{-\infty}^\infty x f_X(x) dx$,  

o ile funkcja $x f_X(x)$ jest całkowalna z modułem.

**Uwaga 7.12**  

Założenie całkowalności z modułem przyjmujemy z przyczyn podobnych jak w przypadku zmiennych dyskretnych. Tak jak poprzednio może ono prowadzić do dość mało intuicyjnych sytuacji. Można na przykład sprawdzić, że zmienna $X$ o tzw. standardowym rozkładzie Cauchy'ego, t.j. o gęstości zadanej wzorem $f_X(x) = \frac{1}{\pi(1+x^2)}$ nie ma wartości oczekiwanej pomimo tego, że jej gęstość jest symetryczna względem zera.

**Uwaga 7.13**  

Powyższa definicja mocno przypomina definicję wartości oczekiwanej dla zmiennych dyskretnych. Nie jest to przypadek odosobniony. Jak wkrótce zobaczymy, większość definicji i twierdzeń dotyczących zmiennych dyskretnych ma swoje odpowiedniki ciągłe. Odpowiedniki te powstają z reguły przez zastąpienie sum całkami, a wyrażeń postaci $P(X=x)$ wyrażeniami $f_X(x)$. Nie jest to specjalnie zaskakujące - o rozkładach ciągłych możemy myśleć jako o granicach rozkładów dyskretnych.

Definicja wariancji dla zmiennych ciągłych jest taka sama jak dla dyskretnych  

**Definicja (wariancja zmiennej ciągłej)**  

Niech $X$ będzie zmienną losową o rozkładzie ciągłym. Wtedy wariancją  $X$ nazywamy  

$Var(X) = E(X-EX)^2$,  

o ile ta wartość oczekiwana istnieje.

Podstawowe własności wartości oczekiwanej i wariancji przenoszą się z przypadku dyskretnego na ciągły. Poniżej omawiamy dwie takie sytuacje.

**Twierdzenie 7.14**  

Niech $X$ będzie zmienną o rozkładzie ciągłym i niech $g:\mathbb{R}\rightarrow\mathbb{R}$ będzie funkcją mierzalną. Wtedy  

$Eg(X) = \int_{-\infty}^\infty g(x) f_X(x) dx$  

o ile $Eg(X)$ istnieje. Ponadto $Eg(X)$ istnieje wtedy i tylko wtedy, gdy funkcja $g(x)f_X(x)$ jest całkowalna z modułem na $\mathbb{R}$.

Nie będziemy dowodzić powyższego twierdzenia - dowód jest dość techniczny. Zwróćmy jednak uwagę na pewną subtelność: nawet jeśli $X$ jest ciągła, to $g(X)$ ciągła być nie musi! We wszystkich interesujących nas sytuacjach $g(X)$ będzie ciągła, ale może być też dyskretna, a nawet, co łatwo sprawdzić, możemy $g$ zdefiniować tak, aby $g(X)$ było "dziwną" zmienną z przykładu 7.10. Wiążą się z tym oczywiście pewne problemy. O ile zdefiniowaliśmy wartość oczekiwaną zarówno dla zmiennych ciągłych jak i dyskretnych, i moglibyśmy podać osobne dowody dla obu sytuacji, o tyle nie mamy pojęcia czym jest wartość oczekiwana zmiennej z przykładu 7.10. Podobnej natury problemy występują także przy innych twierdzeniach omawianych w ramach tego wykładu. Dlatego w większości przypadków zrezygnujemy z podawania pracochłonnych dowodów. Warto jednak zwrócić uwagę, że ogólna ich idea jest z reguły podobna jak w przypadku dyskretnych, szczegóły są jednak dużo bardziej skomplikowane.

Można podać ogólną definicję wartości oczekiwanej, uogólniającą nasze definicje dla zmiennych dyskretnych i ciągłych.  Przy tej ogólnej definicji twierdzenie 7.14 pozostaje prawdziwe, tak jak wiele innych twierdzeń tego wykładu. Niestety nie możemy sobie pozwolić na pełniejsze omówienie tego uogólnienia w ramach naszego wykładu, wymagałoby to od nas dużo dokładniejszego zagłębienia się w teorię miary i całki.

Poniższe twierdzenie jest uogólnieniem wzoru $EX = \sum_{i=1}^\infty P(X \ge i)$ zachodzącego dla zmiennych o wartościach naturalnych na zmienne ciągłe.  

**Twierdzenie 7.15**  

Jeśli $X$ będzie zmienną ciągłą o wartościach nieujemnych i $EX < \infty$. Wtedy $EX = \int_0^\infty (1-F_X(t)) dt = \int_0^\infty P(X \ge t) dt$.

Tym razem, wyjątkowo, podamy dowód.  

**Dowód**  

Tezę twierdzenia uzyskujemy przez prostą zamianę zmiennych:  

$\int_0^\infty P(X \ge t) dt = \int_{t=0}^\infty \int_{s=t}^\infty f_X(s) ds dt = \int_{s=0}^\infty \int_{t=0}^s f_X(s) dt ds =  \int_{s=0}^\infty s f_X(s) ds = EX$.

**Uwaga 7.16**  

Powyższe twierdzenie jest również prawdziwe dla zmiennych dyskretnych (niekoniecznie o wartościach naturalnych). Łatwy dowód pozostawiamy czytelnikowi.

**Przykład 7.17 (wartość oczekiwana rozkładu jednostajnego)**  

Spróbujmy policzyć wartość oczekiwaną zmiennej $X$ o rozkładzie jednostajnym $Unif(a,b)$  

$EX = \int_a^b t \frac{1}{b-a} dt = (\frac{t^2}{2(b-a)})|_a^b = \frac{b^2-a^2}{2(b-a)} = \frac{a+b}{2}$,  

czyli bez niespodzianek.

**Przykład 7.18 (wartość oczekiwana rozkładu wykładniczego)**  

Niech $X \sim Exp(\theta)$. Wtedy, korzystając z twierdzenia 7.15  i wcześniejszego obliczenia $F_X$ mamy  

$EX = \int_0^\infty P(X \ge  t) dt = \int_0^\infty e^{-\theta t} dt = \int_0^\infty (-\frac{e^{-\theta t}}{\theta})' dt = (-\frac{e^{-\theta t}}{\theta})|_0^\infty = 0 - (-\frac{1}{\theta}) = \frac{1}{\theta}$.  

Można też oczywiście obliczyć wartość oczekiwaną wprost z definicji.

W przypadku rozkładu normalnego mamy (ćwiczenia):  

**Fakt 7.19**  

Zmienna $X \sim N(\mu,\sigma^2)$ ma wartość oczekiwaną $EX = \mu$.

---

## Więcej niż jedna zmienna o rozkładzie ciągłym

W tej części wykładu omówimy sytuacje, w których mamy do czynienia z więcej niż jedną zmienną o rozkładzie ciągłym. W szczególności zdefiniujemy pojęcie niezależności ciągłych zmiennych losowych, przyjrzymy się wartości oczekiwanej i wariancji sumy zmiennych, wreszcie uogólnimy pojęcie prawdopodobieństwa warunkowego na nowe sytuacje, które pojawiają się, gdy mamy do czynienia z ciągłymi zmiennymi losowymi.

### Łączny rozkład, łączna dystrybuanta i niezależność ciągłych zmiennych losowych

Nie ma potrzeby definiować na nowo niezależności zmiennych ciągłych. Definicja, której używaliśmy w przypadku zmiennych dyskretnych jest nadal dobra. Przypomnijmy:  

**Definicja (Niezależność zmiennych losowych)**  

Zmienne losowe $X,Y$ są niezależne, jeśli dla każdych zbiorów borelowskich $A,B \subseteq \mathbb{R}$ (lub równoważnie dla każdych  przedziałów $A,B \subseteq \mathbb{R}$ zachodzi $P(X \in A \wedge Y \in B) = P(X \in A) P(Y \in B)$.

W przypadku zmiennych dyskretnych mieliśmy do dyspozycji także prostsze, równoważne sformułowanie niezależności:  

$P(X=a \wedge Y=b) = P(X=a)P(Y=b)$ dla każdych $a,b\in \mathbb{R}$.

W przypadku zmiennych ciągłych powyższe sformułowanie nie jest dobrą charakteryzacją niezależności - obie strony są zawsze równe $0$. Intuicyjnie, powinniśmy zastąpić $P(X=a)$ i $P(Y=b)$ przez $f_X(a)$ i $f_X(b)$, ale czym zastąpić $P(X=a \wedge Y=b)$ ?

**Definicja (łączny rozkład ciągły)**  

Zmienne losowe $X,Y$ mają łączny rozkład ciągły, jeśli istnieje funkcja $f_{X,Y}:\mathbb{R}^2 \rightarrow \mathbb{R}_{\ge 0}$, zwana łączną gęstością $X$ i $Y$ taka, że dla dowolnego mierzalnego zbioru $A \subseteq \mathbb{R}^2$ zachodzi  

$P( (X,Y) \in A) = \int_A f_{X,Y}(x,y) dxdy$.

**Fakt 7.20**  

Jeśli zmienne $X,Y$ mają łączny rozkład ciągły, to $X$ i $Y$ są ciągłe. Ponadto $f_X(x) = \int_{-\infty}^\infty f_{X,Y}(x,y) dy$ oraz $f_Y(y) = \int_{-\infty}^\infty f_{X,Y}(x,y) dx$.

**Dowód**  

Aby pokazać, że zmienna $X$ jest ciągła, wystarczy pokazać, że $f_X$ jak w tezie faktu jest jej gęstością. Niech $B \subseteq \mathbb{R}$ będzie zbiorem mierzalnym. Wtedy  

$\int_B f_X(x) dx = \int_B  \int_{-\infty}^\infty f_{X,Y}(x,y) dy dx = P( (X,Y) \in B\times(-\infty,\infty)) = P(X \in B)$.  

Dowód dla zmiennej $Y$ jest analogiczny.

**Przykład 7.21**  

Nie jest prawdą, że jeśli zmienne losowe $X,Y$ są ciągłe, to są też łącznie ciągłe. Wystarczy wziąć dowolny $X$ o rozkładzie ciągłym, na przykład $X \sim N(0,1)$ oraz $Y=X$. Wtedy dla zbioru $A = \{(x,x):x\in\mathbb{R}\}$ mamy $P((X,Y) \in A) = 1$, ale oczywiście całka z dowolnej funkcji po zbiorze $A$ musi być równa zero, bo zbiór ten ma miarę zero. Przykład ten pokazuje, że łączna ciągłość jest dość mocnym założeniem i często może nie zachodzić. Jak się jednak za chwilę przekonamy, jeśli zmienne $X,Y$ są ciągłe i niezależne, to są też łącznie ciągłe.

**Definicja (łączna dystrybuanta)**  

Łączną dystrybuantą zmiennych losowych $X,Y$ nazywamy funkcję $F_{X,Y}(x,y) = P(X \le x \wedge Y \le y)$.

**Twierdzenie 7.22**  

Jeśli zmienne $X,Y$ mają łączny rozkład ciągły, to $F_{X,Y}$ jest różniczkowalna (prawie wszędzie) i zachodzi (także prawie wszędzie):  

 $f_{X,Y}(x,y) = \frac{\partial\partial F_{X,Y}(x,y)}{\partial x \partial y}$.

Dowód pomijamy.

Jeśli $X,Y$ są niezależne i łącznie ciągłe, to różniczkując tożsamość $F_{X,Y}(x,y) = F_X(x)F_Y(y)$ dwukrotnie (po $x$ i po $y$) dostajemy $f_{X,Y}(x,y) = f_X(x) f_Y(y)$. Oczywiście nie dowodzi to tego, że niezależne zmienne ciągłe są łącznie ciągłe, ale sugeruje w jaki sposób można opisać niezależność za pomocą gęstości.

**Twierdzenie 7.23**  

Niech $X,Y$ - zmienne o rozkładzie ciągłym. Wtedy $X,Y$ są niezależne wtedy i tylko wtedy, gdy są łącznie ciągłe z gęstością $f_{X,Y}(x,y)$ taką, że $f_{X,Y}(x,y) = f_X(x)f_Y(y)$ (prawie wszędzie).

**Dowód**  

Jeśli $X$ i $Y$ są niezależne, to dla dowolnych przedziałów $A,B \subseteq \mathbb{R}$ zachodzi:  

$P(X \in A \wedge Y \in B) = P(X \in A) P (Y \in B) = \int_A f_X(x) dx \int_B f_Y(y)dy = \int_{A \times B} f_X(x)f_Y(y) dxdy$,  

a zatem $f_{X,Y}(x,y) = f_X(x)f_Y(y)$ jest łączną gęstością $X$ i $Y$.

Z drugiej strony jeśli $X,Y$ są łącznie ciągłe i  $f_{X,Y}(x,y) = f_X(x)f_Y(y)$ prawie wszędzie, to dla dowolnych przedziałów $A,B \subseteq \mathbb{R}$ całkując obie strony po $A\times B$ dostajemy  

$P(X \in A \wedge Y \in B) = P(X \in A)P(Y \in B)$.

Sprawdźmy teraz jak wygląda gęstość sumy niezależnych zmiennych ciągłych:  

**Twierdzenie 7.24**  

Jeśli $X,Y$ są niezależnymi zmiennymi ciągłymi i $Z = X+Y$, to $Z$ jest ciągła i $f_Z(z) = \int_{-\infty}^\infty f_X(x) f_Y(z-x)dx$.

**Dowód**  

Wiemy, że $X,Y$ są niezależne, więc są też łącznie ciągłe z gęstością $f_{X,Y}(x,y) = f_X(x) f_Y(y)$. A zatem  

$P(Z \le a) = P(X+Y \le a) = \int_{x+y \le a} f_X(x) f_Y(y) dxdy$.  

Zmieńmy zmienne na $z=x+y$ i $x$. Mamy wtedy  

$P(Z \le a) = \int_{-\infty}^a \int_{-\infty}^\infty f_X(x) f_Y(s-x) dx ds = \int_{-\infty}^a (\int_{-\infty}^\infty f_X(x) f_Y(s-x) dx) ds$.  

A zatem wewnętrzna całka jest gęstością $Z$, co kończy dowód.

**Przykład 7.25**  

Jako przykładowe zastosowanie pokażemy, że suma dwóch niezależnych zmiennych o rozkładzie normalnym ma też rozkład normalny. Ogólny przypadek  

tego faktu jest dość uciążliwy w dowodzie, dlatego ograniczymy sie do przypadku $X \sim Y \sim N(0,1)$. Niech $Z = X+Y$, wtedy na mocy twierdzenia 7.24 mamy (wszystkie całki są po całej osi rzeczywistej):  

$f_Z(z) = \int \frac{1}{2\pi} e^{-\frac{x^2+(z-x)^2}{2}}dx = \frac{1}{2\pi} \int e^{-\frac{ (\sqrt{2}x - \frac{z}{\sqrt{2}})^2 + \frac{z^2}{2}}{2}} dx$.  

Wstawiamy $y = \sqrt{2}x + \frac{z}{\sqrt{2}}$ (czyli $dx = \frac{dy}{\sqrt{2}}$ i otrzymujemy:  

$f_Z(z) = \frac{1}{2\pi} \int \frac{1}{\sqrt{2}} e^{-\frac{ y^2 + \frac{z^2}{2}}{2}} dy = \frac{1}{2\sqrt{\pi}} e^{-\frac{z^2}{4}} \int \frac{1}{\sqrt{2\pi}} e^{-\frac{ y^2}{2}} dy = \frac{1}{2\sqrt{\pi}}e^{-\frac{z^2}{4}}$.  

W ostatnim przejściu całka jest równa $1$ bo jest gęstością standardowego rozkładu normalnego. Łatwo zauważyć, że otrzymany wyrażenie opisuje gęstość rozkładu $N(0,2)$.

Zachodzi też ogólniejszy  

**Fakt 7.26**  

Jeśli $X \sim N(\mu_1,\sigma_1^2)$ i $Y \sim N(\mu_2,\sigma_2^2)$, to $Z = X+Y$ ma rozkład $N(\mu_1+\mu_2,\sigma_1^2+\sigma_2^2)$.

Dowód jest analogiczny, ale rachunki trochę bardziej skomplikowane. Wystarczy ograniczyć się do przypadku $X\sim N(0,1)$ i $Y \sim N(0,\sigma^2)$ ze względu na następujący bardzo prosty fakt (ćwiczenia)  

**Fakt 7.27**  

Jeśli $X \sim N(\mu,\sigma^2)$, to $cX+a \sim N(c\mu+a,c^2\sigma^2)$.

### Wartość oczekiwana i wariancja sumy zmiennych o rozkładzie ciągłym

Tak jak w przypadku zmiennych dyskretnych zachodzi następujące twierdzenie (dowód pominiemy)  

**Twierdzenie 7.28 (liniowość wartości oczekiwanej)**  

Jeśli $X,Y$ są zmiennymi ciągłymi i istnieje $EX$ i $EY$, to istnieje też $E(X+Y)$ i zachodzi $E(X+Y) = EX+EY$.

Z powyższego twierdzenia, podobnie jak w przypadku zmiennych dyskretnych natychmiast otrzymujemy znany nam przydatny  

wzór na obliczanie wariancji.  

**Fakt 7.29 (wzór na wariancję)**  

Jeśli zmienna ciągła $X$ ma wariancję, to $VarX = E(X^2) - (EX)^2$.

**Przykład 7.30 (wariancja zmiennej o rozkładzie jednostajnym)**  

Niech $X \sim Unif(a,b)$. Spróbujmy obliczyć $Var(X)$ korzystając ze wzoru $VarX = E(X^2) - (EX)^2$. Mamy  

$E(X^2) = \int_{a}^b x^2 \frac{1}{b-a} dx = (\frac{x^3}{3(b-a)})|_a^b = \frac{b^3-a^3}{3(b-a)} = \frac{a^2+ab+b^2}{3}$.  

Ponadto wiemy już, że  

$(EX)^2 = \frac{(a+b)^2}{4} = \frac{a^2 +2ab+b^2}{4}$.  

A zatem  

$VarX = \frac{4a^2+4ab+4b^2}{12}-\frac{3a^2+6ab+3b^2}{12} = \frac{a^2-2ab+b^2}{12} = \frac{(a-b)^2}{12}$.

**Przykład 7.31 (wariancja zmiennej o rozkładzie wykładniczym)**  

Niech $X \sim Exp(\theta)$. Obliczymy $Var(X)$ korzystając, jak poprzednio, ze wzoru $VarX = E(X^2) - (EX)^2$. Mamy  

$E(X^2) = \int_{0}^\infty x^2 \theta e^{-\theta x} dx = \int_{0}^\infty x^2 (-e^{-\theta x})' dx$.  

Ze wzoru na całkowanie przez części dostajemy  

$E(X^2) = (-x^2 e^{-\theta x})|_0^\infty + 2\int_{0}^\infty x e^{-\theta x} dx = 0-(-0) + 2 \frac{1}{\theta}EX = \frac{2}{\theta^2}$.  

Stąd  

$Var(X) = \frac{2}{\theta^2} - \frac{1}{\theta^2} = \frac{1}{\theta^2}$.

Można też pokazać (ćwiczenia), że  

**Twierdzenie 7.32**  

Jeśli $X \sim N(\mu,\sigma^2)$, to $VarX = \sigma^2$.

Tak jak w przypadku zmiennych dyskretnych, wartość oczekiwana ogólnie nie jest multiplikatywna, a wariancja addytywna, ale:  

**Twierdzenie 7.33**  

Jeśli $X,Y$ są niezależnymi zmiennymi ciągłymi i istnieje $EX$ i $EY$, to istnieje też  $E(XY)$ i zachodzi $E(XY) = EXEY$.  

**Twierdzenie 7.34**  

Jeśli $X,Y$ są niezależnymi zmiennymi ciągłymi i istnieje $VarX$ i $VarY$, to istnieje też  $Var(X+Y)$ i $Var(X+Y)=VarX+VarY$.

Dowód pierwszego z tych twierdzeń pominiemy, drugie wynika z pierwszego w sposób analogiczny jak dla zmiennych dyskretnych.

### Prawdopodobieństwo warunkowe

W prawdopodobieństwach warunkowych zdarzeń definiowanych przez zmienne ciągłe nie ma w większości przypadków niczego niezwykłego i możemy je obliczać standardowymi sposobami, korzystając ze znanych nam definicji. Dotyczy to na przykład prawdopodobieństw postaci $P(X \in A | B)$ czy $P(X \in A | Y \in B)$, o ile $P(Y \in B) > 0$.  Możemy też korzystając z definicji z wykładu o zmiennych dyskretnych zdefiniować *warunkowy rozkład ciągłej zmiennej losowej*

Nie jest jednak jasne co zrobić z prawdopodobieństwem warunkowym postaci $P(X \in A | Y = y)$. Z jednej strony możemy często chcieć obliczać wartość tego wyrażenia. Możemy na przykład chcieć zapytać o to jakie jest prawdopodobieństwo tego, że losowa osoba waży co najmniej 80kg, jeśli wiemy, że ma 180cm wzrostu, itp. Z drugiej strony, jeśli spróbujemy obliczyć wartość tego wyrażenia za pomocą znanej nam definicji, to otrzymamy iloraz  

$P(X \in A | Y=y) = \frac{P(X \in A \wedge Y=y)}{P(Y=y)}$,  

w którym zarówno licznik i jak i mianownik są równe 0.

**Definicja (gęstość warunkowa)**  

Niech $X$ i $Y$ będą zmiennymi o łącznym rozkładzie ciągłym z gęstością $f_{X,Y}$ i niech $f_Y$ będzie gęstością $Y$. Jeśli  $y \in \mathbb{R}$ jest taki, że $f_Y(y) \neq 0$, to gęstością warunkową $X$ pod warunkiem $Y=y$ nazywamy funkcję  

$f_{X|Y=y}(x) = \frac{f_{X,Y}(x,y)}{f_Y(y)}$.

**Definicja (prawdopodobieństwo warunkowe)**  

Przy założeniach jak wyżej i dla dowolnego mierzalnego $A \subseteq \mathbb{R}$, prawdopodobieństwem warunkowym $X \in A$ pod warunkiem $Y=y$ nazywamy  

$P(X \in A | Y=y) = \int_A f_{X|Y=y}(x) dx$.

Zauważmy przede wszystkim, że $f_{X|Y=y}$ jest funkcją gęstości, t.j. całka z $f_{X|Y=y}$ po całej osi rzeczywistej wynosi $1$. Wynika to natychmiast z faktu 7.20.

Dlaczego tak właśnie zostało zdefiniowane prawdopodobieństwo $P(X \in A | Y=y)$? Istnieją co najmniej 2 intuicyjne sposoby "wyprowadzenia" tej definicji. Po pierwsze: jeśli wiemy, że $Y=y_0$, to patrzymy na gęstość $f_{X,Y}(x,y)$ ograniczoną do $y=y_0$, czyli po prostu $f_{X,Y}(x,y_0)$. Chcielibyśmy użyć tej funkcji jako gęstości, ale nie całkuje się ona na $\mathbb{R}$ do 1. Łatwo to jednak naprawić skalując ją czynnikiem $f_Y(y)$.

Drugie intuicyjne wyprowadzenie mogłoby wyglądać tak: skoro nie wiemy jak obliczyć $P(X \in A | Y=y)$, to obliczmy $P(X \in A | Y \in I_y)$ dla małego przedziału $I_y$ zawierającego $y$. Jeśli ten przedział jest na tyle mały, żeby zarówno $f_Y$ jak i $f_{X,Y}(x,y)$ dla każdego ustalone $x$ była na nim prawie stała (pomijamy to czy taki przedział musi istnieć, w końcu szukamy tylko intuicji), to dostajemy:  

$P(X \in A | Y \in I_y) = \frac{\int_{s \in A} \int_{t \in I_y} f_{X,Y}(s,t) dt ds}{ \int_{t \in I_y} f_Y(t) dt} \approx \frac{\int_{s \in A} |I_y| f_{X,Y}(s,y)ds}{|I_y| f_Y(y)} = \int_{s \in A} \frac{f_{X,Y}(s,y)}{f_Y(y)} ds$,  

czyli dokładnie to czego się spodziewaliśmy.

Zdefiniowane przez nas prawdopodobieństwo warunkowe ma własności analogiczne do zwykłego prawdopodobieństwa warunkowego, np.  

**Twierdzenie 7.35 (Wzór na prawdopodobieństwo całkowite)**  

Jeśli $X,Y$ są ciągłe i łącznie ciągłe, a $A \subseteq \mathbb{R}$ jest mierzalny, to zachodzi  

$P(X \in A) = \int_{-\infty}^{\infty} P(X\in A | Y=y)  f_Y(y) dy$.

Dowód wynika natychmiast z definicji gęstości warunkowej.

---

## Centralne Twierdzenie Graniczne

Ostatnią część tego wykładu poświęcimy zapowiadanemu wcześniej Centralnemu Twierdzeniu Granicznemu. Twierdzenie to mówi, że rozkład sumy wielu niezależnych zmiennych o tym samym rozkładzie jest bliski normalnemu (tak naprawdę rozkłady mogą być różne, ważne jest aby mały zbiór zmiennych nie dominował sumy, skoncentrujemy się jednak na najprostszej wersji twierdzenia).

Zastanówmy się jak mogłoby wyglądać to twierdzenie. Niech $X_1,X_2,\ldots$ będzie ciągiem niezależnych zmiennych o tym samym rozkładzie. Chciałoby się powiedzieć, że rozkład $Z_n = \sum_{i=1}^n X_i$ zbiega do rozkładu normalnego wraz z rosnącym $n$, ale takie twierdzenie oczywiście nie może być prawdziwe, bo jeśli na przykład $\mu = EX_1 = EX_2 = \ldots$ istnieje i jest większe od zera, to kolejne $Z_n$ będą miały coraz większe wartości oczekiwane i nie mogą do niczego zbiegać.

Może w takim razie załóżmy istnienie $\mu = EX_1 = EX_2 = \ldots$ i popatrzmy na graniczne zachowanie $Z_n = \sum_{i=1}^n (X_i - \mu)$. Tutaj mamy $EZ_n = 0$ dla każdego $n$, ale niestety jeśli $\sigma^2 = Var(X_1) = Var(X_2) = \ldots$ istnieje to $Var(Z_n)$ są coraz większe i tak jak poprzednio ciąg $Z_n$ nie może do niczego zbiegać. Musimy znormalizować $Z_n$ tak, aby wszystkie $Z_n$ miały tę samą wariancję, najprościej dzieląc przez $\sqrt{n}\sigma$. Dlatego Centralne Twierdzenie Graniczne formułujemy tak:

**Twierdzenie 7.36 (Centralne Twierdzenie Graniczne (CTG))**  

Niech $X_1,X_2,\ldots$ będzie ciągiem niezależnych zmiennych losowych o tym samym rozkładzie, wartości oczekiwanej $\mu$ i wariancji $\sigma^2>0$.  Niech ponadto $Z_n = \frac{\sum_{i=1}^n (X_i - \mu)}{\sqrt{n}\sigma}$. Wtedy rozkład $Z_n$ zbiega do rozkładu $N(0,1)$ w następującym sensie:  

$\forall_{z \in \mathbb{R}} \lim_{n \rightarrow \infty} P(Z_n \le z) = \Phi(z)$,  

gdzie $\Phi$ jest dystrybuantą rozkładu $N(0,1)$.
