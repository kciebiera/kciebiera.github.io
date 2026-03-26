---
title: "Ćwiczenia 4: Dyskretne zmienne losowe"
source_url: "http://smurf.mimuw.edu.pl/node/466"
source_kind: "html"
---

<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@4/tex-mml-chtml.js"></script>

# Ćwiczenia 4: Dyskretne zmienne losowe

# Rozkład dwumianowy i Poissona

## Zadanie 1 (Kształt rozkładu dwumianowego)

Rozważmy zmienną $X \sim \textrm{Binom}(n,p)$. Niech $K = \lfloor (n+1)p \rfloor$. Pokazać, że $P(X=k)$ jest funkcją niemalejącą dla $k \le K$, oraz malejącą dla $k \ge K$.

## Zadanie 2 (Kształt rozkładu Poissona)

Rozważmy zmienną $X \sim \textrm{Pois}(\lambda)$. Niech $K = \lfloor \lambda \rfloor$. Pokazać, że $P(X=k)$ jest funkcją rosnącą dla $k \le K$, oraz malejącą dla $k \ge K$.

## Zadanie 3 (Rozkład Poissona jako granica rozkładów dwumianowych)

Pokazać, że jeśli $n_i \rightarrow \infty$ oraz $n_i p_i \rightarrow \lambda$ (a zatem $p_i \rightarrow 0$ ) to rozkłady $\textrm{Binom}(n_i,p_i)$  zbiegają do rozkładu $\textrm{Pois}(\lambda)$.

**Uwaga:** Chodzi tu o zbieżność punktową, tzn. zbieżność prawdopodobieństw przyjęcia każdej konkretnej wartości.

## Zadanie 4

Idziemy na przyjęcie na którym jest 500 osób. Jakie jest prawdopodobieństwo tego, że dokładnie dwie osoby będą miały tę samą datę urodzin (tj. miesiąc/dzień) co my? Rozwiązać na dwa sposoby:

- użyć rozkładu dwumianowego,
- przybliżyć rozkładem Poissona.

## Zadanie 5 (trudne)

Gramy serię gier (mecz) z przeciwnikiem od którego jesteśmy słabsi, tzn. nasze prawdopodobieństwo wygrania pojedynczej gry jest równe $p < 0.5$. Mecz składa się z **parzystej** liczby gier, wygrywamy jeśli wygramy **więcej** niż połowę gier. Możemy wybrać liczbę gier w meczu: 0, 2,4,6, itd.  Jaką liczbę powinniśmy wybrać, aby zmaksymalizować prawdopodobieństwo zwycięstwa?

Wskazówka:  

 Łatwo uzasadnić, że jeśli zagramy bardzo dużo gier to nasze prawdopodobieństwo wygrania zbiega do 0. Podobnie jeśli zagramy 0 gier (musimy wygrać **więcej** niż połowę!). Co więcej, dość łatwo się przekonać, że zagranie czterech gier może być bardziej opłacalne niż dwóch. A zatem optymalną liczbą gier jest taka liczba $n$, że zagranie $n$ gier daje większe prawdopodobieństwo zwycięstwa, niż zagranie $n-2$ lub $n+2$ gier. Sformułuj i znajdź rozwiązanie odpowiednich nierówności.

## Zadanie 6 (Zapałki Banacha)

Palący matematyk nosi po jednym pudełku zapałek w lewej i prawej kieszeni spodni. Za każdym razem, gdy chce zapalić, wyciąga pudełko z losowej kieszeni. Jakie jest prawdopodobieństwo tego, że gdy wyciągnie w końcu puste pudełko, w drugim jest dokładnie $k$ zapałek? Zakładamy, że zabawa zaczyna się z dwoma pudełkami po $n$ zapałek każde.  

**Uwaga:** Uzyskany rozkład prawdopodobieństwa nazywa się ujemnym rozkładem dwumianowym. Czy widzisz dlaczego?

# Rozkład geometryczny

## Zadanie 1 (Własność braku pamięci)

Pokazać, że zmienna o rozkładzie geometrycznym "nie ma pamięci", tzn. dla dowolnych $0\le m < n$  zachodzi $P(X = n| X > m) = P( X = n-m)$. Nieco mniej formalnie: jeśli czekamy na orła i wypadło już $m$ kolejnych reszek, to prawdopodobieństwo tego, że pierwszy orzeł wypadnie  za $n-m$ rzutów jest takie samo jak gdyby całej przeszłości (t.j. $m$ reszek) nie było. Wiele osób sądzi, że $P(X=n| X>m) > P(X=n-m)$ - orzeł niejako "należy się".

## Zadanie 2 (Brak pamięci definiuje rozkład geometryczny)

Pokaż, że każda zmienna przyjmująca wartości $1,2,\ldots$ i nie mająca pamięci (zobacz poprzednie zadanie) ma rozkład geometryczny. Jest to więc własność definiująca rozkład geometryczny.

## Zadanie 3

Rzucamy monetą do momentu, kiedy wypadnie drugi orzeł. Pokazać, że jeśli ten drugi orzeł wypada w $n$-tym rzucie, prawdopodobieństwo wypadnięcia pierwszego orła w $i$-tym rzucie ($i=1,\ldots,n-1$)jest takie samo dla każdego $i$.

# Niezależność zmiennych losowych

## Zadanie 1 (Stabilność rozkładu dwumianowego i Poissona)

Jaki rozkład ma suma dwóch niezależnych zmiennych losowych o rozkładzie dwumianowym z tym samym $p$? Jaki rozkład ma suma dwóch niezależnych zmiennych losowych o rozkładzie Poissona?

## Zadanie 2

Pokazać, że jeśli $X$ i $Y$ są niezależne i $f,g:  \mathbb{R} \rightarrow \mathbb{R}$, to $f(X)$ i $g(Y)$ są niezależne.

## Zadanie 3 ("Dwumianowe przerzedzanie" rozkładu Poissona)

Jaki jest rozkład liczności potomstwa owada, u którego liczba złożonych jaj ma rozkład Poissona, i z każdego z jaj niezależnie wykluwa się młode z prawdopodobieństwem p?

## Zadanie 4

W sytuacji jak w poprzednim zadaniu pokazać, że zmienne losowe opisujące wyklute i niewyklute jaja są niezależne.

## Zadanie 5

Niech $X, Y$ będą niezależne o rozkładzie Poissona. Pokazać, że rozkład $X | X+Y=n$ jest dwumianowy, t.j. $P(X=k|X+Y=n) = P(Z=k)$ dla pewnej zmiennej $Z$ o rozkładzie dwumianowym.

## Zadanie 6

Pokazać, że jeśli $X,Y$ są niezależnymi zmiennymi o rozkładzie geometrycznym, to $\min(X,Y)$ też ma rozkład geometryczny.
