---
layout: "default"
title: "Wykład 6: Nierówności probabilistyczne"
source_url: "http://smurf.mimuw.edu.pl/node/711"
source_kind: "html"
uses_math: true
---

# Wykład 6: Nierówności probabilistyczne

## Szacowanie ogonów

Zdefiniowane w poprzednim wykładzie pojęcia wartości oczekiwanej umożliwia sformułowanie w języku rachunku prawdopodobieństwa następujących naturalnych pytań

- Ile średnio wypada orłów w $N$ rzutach symetryczną monetą?
- Ile średnio urn będzie pustych, jeśli wrzucimy losowo $N$ kul do $N$ urn?
- Jak długo średnio działa algorytm Quicksort dla losowej permutacji? (ew. jak duża może być ta średnia dla konkretnej permutacji jeśli element dzielący jest wybierany losowo)
- Jak głębokie jest średnio drzewo BST powstałe przez wstawienie do początkowo pustego drzewa losowej permutacji elementów $1,\ldots,n$?

i wielu innych. Sprowadzają się one do znalezienia wartości oczekiwanej odpowiednio zdefiniowanej zmiennej losowej.

W poprzednim wykładzie zdefiniowaliśmy także pojęcia wariancji i odchylenia standardowego, mierzące rozmiar odchyleń zmiennej losowej od jej wartości oczekiwanej. Dla rozkładów, dla których obliczyliśmy wariancję (rozkład dwumianowy, Poissona i geometryczny) te typowe odchylenia są małe, istotnie mniejsze niż wartość oczekiwana. Powinniśmy się więc spodziewać, że z dużym prawdopodobieństwem zmienne losowe będą przyjmować wartości bliskie swoim wartościom oczekiwanym. Na przykład, w $N$ rzutach monetą nie tylko średnio wypada $\frac{N}{2}$ orłów, ale też z dużym prawdopodobieństwem liczba orłów jest niezbyt odległa od $\frac{N}{2}$.  Postaramy się teraz pokazać podstawowe metody dowodzenia tego rodzaju stwierdzeń.

Wartości zmiennej losowej odległe od jej wartości oczekiwanej nazywa się z reguły *ogonami*. Można więc powiedzieć, że naszym celem jest szacowanie prawdopodobieństw ogonów. Często mówi się też po prostu o *szacowaniu ogonów*.

---

## Nierówność Markowa

Jedną z najprostszych i najbardziej ogólnych metod szacowania ogonów jest nierówność Markowa  

**Twierdzenie 6.1 (Nierówność Markowa)**  

Niech  $X$ będzie dyskretną zmienną losową o wartościach nieujemnych i niech $EX < \infty$. Wtedy dla dowolnego $c > 0$  

$P(X \ge c EX) \le \frac{1}{c}$  

**Dowód**  

Z definicji $EX$ mamy  

$EX = \sum_x x P(X=x) \ge \sum_{x \ge cEX} xP(X=x) \ge c EX \cdot P(X \ge cEX)$,  

z czego wynika teza.

Powyższy dowód ma bardzo prostą intuicję fizyczną: Jeśli o $EX$ myśleć jako o środku ciężkości masy rozłożonej zgodnie z $P$, to na prawo od $cEX$ nie może być tej masy więcej niż $\frac{1}{c}$, bo nie dałoby się jej w żaden sposób zrównoważyć. Widać, że kluczowe jest tu założenie o nieujemności zmiennej $X$. Gdyby zmienna $X$ mogła przyjmować ujemne wartości to moglibyśmy do równoważenia użyć masy położonej daleko na lewo od $0$.

**Uwaga 6.2**  

Często podaje się nierówność Markowa w równoważnym sformułowaniu:  

$P(X \ge c) \le \frac{EX}{c}$

**Przykład 6.3**  

Jakie jest prawdopodobieństwo uzyskania w $n=1000$ rzutach co najmniej $\frac{3}{4}N = 750$ orłów? Wydaje się, że prawdopodobieństwo to powinno być bardzo małe. Zobaczmy jakie oszacowanie uzyskamy korzystając z nierówności Markowa.

Niech $X_1,\ldots,X_n$ będą wynikami poszczególnych rzutów, przy czym wartość 1 oznacza orła, a 0 - reszkę. Wtedy $X = \sum_{i=1}^n X_i$ jest łączną liczbą orłów. Ponieważ $EX = \frac{N}{2}$, to z nierówności Markowa dostajemy niezbyt imponujące oszacowanie:  

$P(X \ge \frac{3}{4}N) = P(X \ge \frac{3}{2} EX) \le \frac{2}{3}.$

Poniższy przykład pokazuje, że nierówność Markowa nie jest zbyt mocna. Pomimo tego jest to nierówność ważna z co najmniej dwóch powodów. Po pierwsze, wymaga bardzo słabych założeń: nieujemności i istnienia wartości oczekiwanej. Czasem dysponujemy jedynie wartością oczekiwaną i nierówność Markowa jest jedynym narzędziem jakiego możemy użyć. Po drugie, jak się wkrótce przekonamy, aplikując nierówność Markowa do odpowiednio zdefiniowanej zmiennej można uzyskać nierówności dużo mocniejsze.

---

## Nierówność Czebyszewa

Nierówności Markowa nie da się poprawić, jeżeli dysponujemy jedynie wartością oczekiwaną. Niech na przykład $X$ będzie zmienną zdefiniowaną następująco: $P(X=c) = \frac{1}{c}$ oraz $P(X=0) = 1-\frac{1}{c}$, gdzie $c > 1$. Wtedy $EX = 1$ i $P(X \ge cEX) = \frac{1}{c}$.  

Zauważmy jednak, że tak określona zmienna jest skoncentrowana w dwóch punktach - swoich ekstremach. W szczególności, wartości zmiennej $X$ mają bardzo duży "rozrzut". Można się spodziewać, że dla zmiennych o małym odchyleniu standardowym, a więc dużo bardziej skoncentrowanych wokół wartości oczekiwanej, powinniśmy być w stanie uzyskać dużo lepsze oszacowanie.

**Twierdzenie 6.4 (Nierówność Czebyszewa)**  

Niech $X$ będzie dyskretną zmienną losową taką, że $EX < \infty$ oraz $VarX < \infty$ i niech $\sigma = \sqrt{VarX}$ będzie odchyleniem standardowym $X$. Wtedy dla dowolnego $c > 0$  

$P(\vert{}X-EX\vert{}\ge c\sigma) \le \frac{1}{c^2}$.  

**Dowód**  

Niech $Y=(X-EX)^2$. Wtedy korzystając z nierówności Markowa dostajemy  

$P( \vert{}X-EX\vert{} \ge c\sigma ) = P( (X-EX)^2\ge c^2 VarX)  = P( Y \ge c^2 EY) \le 1/c^2$.

**Uwaga 6.5**  

Tak jak w przypadku nierówności Markowa, często używa się alternatywnego sformułowania nierówności Czebyszewa:  

$P(\vert{}X-EX\vert{}\ge c) \le \frac{VarX}{c^2}$.

**Przykład 6.3 (c.d.)**  

Zobaczmy jakie oszacowanie można uzyskać za pomocą nierówności Czebyszewa.

Zmienna $X$ ma rozkład Binom$(n,p=\frac{1}{2})$, a zatem $VarX = npq=\frac{n}{4}$. Korzystając z alternatywnej wersji nierówności Czebyszewa dostajemy:  

$P(X \ge \frac{3}{4}n) = \frac{1}{2} P( \vert{}X-EX\vert{} \ge \frac{1}{4}n) \le \frac{1}{2} \frac{\frac{n}{4}}{(\frac{1}{4}n)^2} = \frac{2}{n}$  

(w pierwszym kroku skorzystaliśmy z tego, że $P(X \ge \frac{3}{4}n) = P(X \le \frac{1}{4}n)$ ).

Jest to oszacowanie dużo lepsze niż $P(X \ge \frac{3}{4}n) \le \frac{2}{3}$ uzyskane z nierówności Markowa. Jak jednak wkrótce zobaczymy, jest ono wciąż bardzo daleko od prawdziwej wartości tego prawdopodobieństwa.

Zanim przejdziemy do kolejnej nierówności, udowodnimy bardzo ważny wniosek z nierówności Czebyszewa  

**Twierdzenie 6.6 (Słabe Prawo Wielkich Liczb (SPWL))**  

Niech zmienne losowe $X_1,X_2,\ldots$ będą niezależne o tym samym rozkładzie $X$ i niech $\bar{X_n} =\frac{\sum_{i=1}^n X_i}{n}$ dla $n =1,2,\ldots$ . Niech ponadto $\mu =  EX < \infty$ i $\sigma^2 = Var X < \infty$. Wtedy dla każdego $\varepsilon > 0$  

$\lim_{n\rightarrow\infty} P(\vert{} \bar{X_n}-\mu\vert{} > \varepsilon) = 0$.

**Dowód**  

Z liniowości wartości oczekiwanej dla wszystkich $n=1,2,\ldots$ mamy $E\bar{X_n} = \mu$. Ponadto z niezależności $X_i$ wynika, że $Var\bar{X_n} = \frac{1}{n^2} \cdot n \sigma^2 = \frac{\sigma^2}{n}$. A zatem z nierówności Czebyszewa dostajemy  

$P(\vert{} \bar{X_n}-\mu\vert{} > \varepsilon) \le \frac{\frac{\sigma^2}{n}}{\varepsilon^2} \rightarrow_{n\rightarrow\infty} 0$,  

co kończy dowód.

**Uwaga 6.7**  

Założenie, że zmienne $X_i$ mają skończoną wariancję nie jest konieczne, ale bez niego nie możemy użyć nierówności Czebyszewa i dowód istotnie się komplikuje.

Prawdziwe jest również następujące twierdzenie:  

**Twierdzenie 6.8 (Mocne Prawo Wielkich Liczb (MPWL))**  

Przy założeniach jak w twierdzeniu 6.6 zachodzi  

$P(\lim_{n\rightarrow\infty} \bar{X_n} = \mu) = 1$.

Twierdzenie to pozostawimy bez dowodu. Polecamy jednak czytelnikowi zastanowienie się nad tym dlaczego MPWL jest silniejszym twierdzeniem niż SPWL?

### Intuicja częstościowa

Rozważmy nieskończony ciąg niezależnych powtórzeń tego samego doświadczenia modelowanego przestrzenią $(\Omega,P)$. Niech $A \subseteq \Omega$ i niech zdarzenia $A_1,A_2,\ldots$ odpowiadają zajściu $A$ w doświadczeniach $1,2,\ldots$. Niech ponadto zmienne $X_1,X_2$ będą zdefiniowane następująco: $X_i(\omega) = [\omega \in A_i]$, t.j. zmienna $X_i$ jest równa $1$ jeśli $A_i$ zaszło i $0$ wpp.

W naszych dotychczasowych rozważaniach dwukrotnie już pojawiała się tzw. intuicja częstościowa, którą można opisać następującą równością:  

$\lim_{n\rightarrow\infty} \frac{\sum_{i=1}^n X_i}{n} = P(A)$.  

Używaliśmy tej intuicji dobierając modele probabilistyczne w wykładzie 1 i definiując wartość oczekiwaną w wykładzie 5. Dlatego należy traktować jako dobrą wiadomość to, że intuicję tę można w naszej teorii "udowodnić" aplikując MPWL do zmiennych $X_1,X_2,\ldots$  (zachęcamy czytelnika do sprawdzenia tego faktu).

---

## Nierówność Chernoffa

Wzorując się na dowodzie nierówności Czebyszewa możemy próbować aplikować nierówność Markowa do dowolnej zmiennej $Y = f(X)$, o ile tylko funkcja $f$ jest nieujemna i monotonicznie rosnąca:  

$P(X \ge c) = P(f(X) \ge f(c)) \le \frac{Ef(X)}{f(c)}$.  

(w pierwszym kroku korzystamy z monotoniczności $f$, a w drugim z nierówności Markowa, do której jest potrzebna nieujemność $f$).

Oczywiście takie postępowanie ma sens jedynie jeśli:

1. jesteśmy w stanie obliczyć bądź oszacować $Ef(X)$, oraz
2. uzyskane w ten sposób oszacowanie jest lepsze niż znane nam nierówności.

Okazuje się, że oba te warunki są spełnione, jeśli przyjmiemy $f(x) = e^{tx}$, gdzie $t > 0$ jest parametrem, który ustalimy później. W szczególności, podstawą naszych dalszych rozważań będzie następujące twierdzenie:  

**Twierdzenie 6.9 (Ogólna nierówność typu Chernoffa)**  

Dla dowolnej zmiennej losowej $X$ zachodzi  

$P(X \ge c) \le  \min_{t>0} \frac{E(e^{tX})}{e^{tc}}$  

oraz  

$P(X \le c) \le \min_{t<0} \frac{E(e^{tX})}{e^{tc}}$.

**Dowód**  

Aby uzyskać pierwszą nierówność powtarzamy rozumowanie przeprowadzone na początku tego paragrafu dla funkcji $f(x) = e^{tx}$ i otrzymujemy dla dowolnego $t > 0$  

$P(X \ge c) =  P(e^{tX} \ge e^{tc}) \le \frac{E(e^{tX})}{e^{tc}}$.  

Skoro nierówność ta zachodzi dla każdego $t > 0$, to zachodzi też nierówność z tezy twierdzenia.

Drugą nierówność dostajemy w analogiczny sposób. Dla dowolnego $t < 0$ zachodzi bowiem  

$P(X \le c) =  P(e^{tX} \ge e^{tc}) \le \frac{E(e^{tX})}{e^{tc}}$.

Aby uzyskać konkretne oszacowanie na $P(X \ge c)$ lub $P(X \le c)$, bez nieustalonego parametru $t$ i tajemniczego wyrażenia $E(e^{tX})$, musimy zdecydować jaki rozkład ma mieć zmienna $X$. Bardzo ciekawe wyniki można uzyskać w sytuacji opisanej w następującym twierdzeniu:  

**Twierdzenie 6.10 (nierówność Chernoffa dla prób Poissona)**  

Niech $X_1,\ldots,X_n$ niezależne zmienne o rozkładzie 0/1-kowym, przy czym $P(X_i = 1)=p_i = 1-q_i$. Niech ponadto $X = \sum_{i=1}^n X_i$ i niech $\mu = EX = \sum_{i=1}^n p_i$. Wtedy:

1. dla dowolnego $\delta > 0$ zachodzi $P(X \ge (1+\delta)\mu) \le \left(\frac{e^\delta}{(1+\delta)^{1+\delta}}\right)^\mu$
2. dla dowolnego $0 < \delta \le 1$ zachodzi $P(X \ge (1+\delta)\mu) \le e^{-\mu \delta^2 / 3}$
3. dla $\delta \ge 2e-1$ zachodzi $P(X \ge (1+\delta)\mu) \le 2^{-(1+\delta)\mu}$, lub prościej: dla $c \ge 2e\mu$ zachodzi $P(X \ge c) \le 2^{-c}$.

**Uwaga 6.11**  

Ciąg zmiennych $X_i$ jak w twierdzeniu powyżej nazywa się często *próbami Poissona* (nie mylić z rozkładem Poissona!). Jest to uogólnienie prób Bernoulliego na przypadek niekoniecznie równych prawdopodobieństw sukcesu.

**Uwaga 6.12**  

Może dziwić to, że w tezie twierdzenia sformułowane są aż trzy nierówności. Przyczyna jest bardzo prosta. Formułując konkretną nierówność dokonujemy kompromisu między jej siłą, a prostotą zapisu. Nierówność pierwsza jest najmocniejsza i zachodzi dla wszystkich wartości $\delta$, jest jednak najbardziej skomplikowana. Nierówności druga i  trzecia są słabsze i zachodzą tylko dla pewnych przedziałow wartości $\delta$, są jednak zdecydowanie prostsze.

**Przykład 6.3 (c.d.)**  

Zanim przejdziemy do dość technicznego dowodu, sprawdźmy jakie oszacowanie możemy uzyskać korzystając z nierówności Chernoffa dla przykładu z rzutami monetą.  

$P (X \ge \frac{3}{4}n) = P(X \ge (1+\frac{1}{2}) \frac{1}{2}n) \le \left( \frac{e^\frac{1}{2}}{(\frac{3}{2})^\frac{3}{2}}\right)^{\frac{1}{2}n} = (\frac{8e}{27})^{\frac{1}{4}n} \sim 0.95^n.$  

To oszacowanie jest nieporównywalnie lepsze od oszacowania uzyskanego za pomocą nierówności Czebyszewa. Jeśli przyjmiemy $n=1000$ rzutów to dostaniemy:

- Markow: $\frac{2}{3}$,
- Czebyszew: $\frac{2}{n} = 0.002$,
- Chernoff: $\sim 3 \cdot 10^{-24}$.

Zachęceni tak spektakularnym sukcesem przystąpmy do dowodu.

**Dowód (nierówności Chernoffa dla prób Poissona)**  

Wiemy, że zachodzi  

$P(X \ge (1+\delta)\mu) \le  \min_{t>0} \frac{E(e^{tX})}{e^{t(1+\delta)\mu}}.$  

Zajmijmy się członem $E(e^{tX})$. Korzystając z przedstawienia $X$ jako sumy niezależnych zmiennych 0/1-kowych dostajemy  

$E(e^{tX}) = E(e^{\sum_{i=1}^n tX_i}) = E(\prod_{i=1}^n e^{tX_i}) = \prod_{i=1}^n E(e^{tX_i}) = \prod_{i=1}^n (q_i + p_i e^t)) = \prod_{i=1}^n (1 + p_i (e^t-1)) \le \prod_{i=1}^n e^{p_i(e^t-1)} = e^{\mu(e^t-1)}$.  

A zatem  

$P(X \ge (1+\delta)\mu) \le  \min_{t>0} \frac{e^{(e^t-1)\mu}}{e^{t(1+\delta)\mu}} = \min_{t>0} e^{(e^t-1-t(1+\delta))\mu}.$  

Dla jakiej wartości parametru $t$ wyrażenie w wykładniku przyjmuje najmniejszą wartość? Pochodna wykładnika to $(e^t-(1+\delta))\mu$.  

Przyrównując ją do zera dostajemy $t = \ln(1+\delta)$. Podstawiając optymalną wartość $t$ do naszego oszacowania dostajemy:  

$P(X \ge (1+\delta)\mu) \le e^{(1+\delta-1-(1+\delta)\ln(1+\delta))\mu} = \left(\frac{e^\delta}{(1+\delta)^{1+\delta}}\right)^\mu$,  

co dowodzi pierwszej części tezy.

Aby udowodnić drugą część wystarczy pokazać, że dla dowolnego $\delta \in [0,1]$ zachodzi $\delta - (1+\delta)\ln(1+\delta) \le -\frac{\delta^2}{3}$, czyli $\delta - (1+\delta)\ln(1+\delta) + \frac{\delta^2}{3} \le 0$. Niech $f(\delta) = \delta - (1+\delta)\ln(1+\delta) + \frac{\delta^2}{3}$. Wtedy mamy  

$f'(\delta) = 1 - \frac{1+\delta}{1+\delta} - \ln(1+\delta) + \frac{2}{3}\delta = -\ln(1+\delta) + \frac{2}{3}\delta$,  

oraz  

$f''(\delta) = -\frac{1}{1+\delta} + \frac{2}{3}$.  

Łatwo zauważyć, że $f''(\delta) < 0$ dla $0 < \delta  < \frac{1}{2}$ oraz $f''(\delta) > 0$ dla $\frac{1}{2} < \delta$. A zatem, na przedziale $(0,1]$ druga pochodna $f$ najpierw maleje, a potem rośnie. Skoro jednak $f'(0) = 0$ i $f'(1) = -\ln(2)+\frac{2}{3} < 0$, to musi być $f'(\delta) < 0$ na całym przedziale $[0,1]$, czyli $f$ jest malejąca. A ponieważ $f(0) = 0$, to $f$ jest ujemna na całym przedziale $(0,1]$, co kończy dowód drugiej nierówności.

Trzecia część tezy w prosty sposób wynika z pierwszej. Jeśli $1+\delta >= 2e$, to mamy:  

$\left(\frac{e^\delta}{(1+\delta)^{1+\delta}}\right)^\mu \le \left(\frac{e}{1+\delta}\right)^{(1+\delta)\mu} \le 2^{-(1+\delta)\mu},$  

co kończy dowód.

Zachodzą również nierówności dla "dolnych ogonów"  

**Twierdzenie 6.13 (nierówność Chernoffa dla prób Poissona - dolne ogony)**  

Niech $X_1,\ldots,X_n$ niezależne zmienne o rozkładzie 0/1-kowym, przy czym $P(X_i = 1)=p_i = 1-q_i$. Niech ponadto $X = \sum_{i=1}^n X_i$ i niech $\mu = EX = \sum_{i=1}^n p_i$. Wtedy dla dowolnego $0 < \delta < 1$ zachodzi

1. $P(X \le (1-\delta)\mu) \le \left(\frac{e^{-\delta}}{(1-\delta)^{1-\delta}}\right)^\mu$
2. $P(X \le (1-\delta)\mu) \le e^{-\mu \delta^2 / 2}$ (tak, tutaj jest 2, a nie 3).

Dowód, analogiczny do przed chwilą przeprowadzonego, pomijamy.

**Uwaga 6.14**  

Nieprzypadkowo w nierównościach Chernoffa korzysta się z funkcji $f(x) = e^{tx}$. Wyrażenie $E(e^{tX})$, które zmuszeni byliśmy szacować, jest w teorii prawdopodobieństwa dobrze znane i zbadane. Nazywa się je z reguły *funkcją tworzącą momentów $X$* i oznacza $M_X(t)$. Funkcja $M_X(t)$ przypomina trochę znaną nam funkcję tworzącą prawdopodobieństwa, w szczególności dla $X$ o wartościach naturalnych zachodzi:  

$M_X(t) = g_X(e^t)$.  

Funkcji tworzących momentów można jednak używać dla dowolnych zmiennych, niekoniecznie dyskretnych. Z ciekawszych ich własności warto wspomnieć następujące rozwinięcie w szereg potęgowy  

$M_X(t) = E e^{tX} = E (\sum_{k=1}^\infty X^k t^k / k!) = \sum_{k=1}^\infty E (X^k) t^k / k!$,  

które jest możliwe przy pewnych założeniach dotyczących $X$. Okazuje się, że $M_X(t)$ jest wykładniczą funkcją tworzącą ciągu momentów $X$, i stąd jej nazwa.

---

## Uwagi końcowe

Z punktu widzenia zastosowań w informatyce, materiał omówiony w ramach tego rozdziału jest szczególnie ważny. Wiele naturalnych i fundamentalnych pytań pojawiających się w zastosowaniach sprowadza się bowiem do szacowania ogonów. Z tego względu umiejętność sprawnego posługiwania się nierównościami probabilistycznymi jest bardzo przydatna. Należy jednak pamiętać, że najmocniejsza nawet ogólna nierówność nie zastąpi solidnej analizy i dobrego zrozumienia konkretnego problemu. Nierzadko najlepsze oszacowanie prawdopodobieństwa ogonów uzyskuje się wprost, bez użycia nierówności. Przykład takiej sytuacji zobaczymy na ćwiczeniach.
