---
title: "Wykład 5: Wartość oczekiwana i wariancja"
source_url: "http://smurf.mimuw.edu.pl/node/710"
source_kind: "html"
---

<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>

# Wykład 5: Wartość oczekiwana i wariancja

## Intuicja

W poprzednim wykładzie zdefiniowaliśmy zmienną $X$, która opisywała sumę oczek na dwóch kostkach. Jaka jest średnia wartość tej zmiennej? Zanim odpowiemy na to pytanie, zastanówmy się co ono właściwie powinno znaczyć.

Intuicyjnie, jeśli będziemy rzucać parą kostek bardzo dużo razy, to średnia z wyników będzie zbiegać do pewnej wartości i tę wartość można nazwać średnią sumą oczek. Jeśli powtórzylibyśmy rzut dwiema kostkami $n$ razy, to spodziewamy się, że wynik $k$ uzyskamy mniej więcej $P(X=k)n$ razy. A zatem średnia suma z $n$ powtórzeń będzie miała wartość bliską  

$\frac{\sum_{k=2}^{12} P(X=k)nk }{n} = \sum_{k=2}^{12} P(X=k)k$.  

Ta wartość wydaje się być rozsądną definicją średniej wartości $X$.

**Uwaga 5.1**  

W powyższym rozumowaniu uznaliśmy, że jeśli $n$ jest duże, to w $n$ powtórzeniach rzutu dwiema kostkami wynik $k$ uzyskamy mniej więcej $P(X=k)n$ razy. Ogólniej, w $n$ powtórzeniach pewnego doświadczenia zdarzenie $A$ powinno wystąpić mniej więcej $P(A)n$ razy. Warto zwrócić uwagę, że korzystamy tu z intuicji częstościowej prawdopodobieństwa, o której mówiliśmy już w pierwszym wykładzie. W szczególności nasze uzasadnienie definicji wartości oczekiwanej ma raczej charakter nieformalny. Co ciekawe, wkrótce okaże się, że intuicji częstościowej odpowiada twierdzenie w naszej teorii (szczególny przypadek tzw. Prawa Wielkich Liczb).

---

## Definicja

**Definicja (Wartość oczekiwana)**  

Niech $X$ będzie zmienną losową o rozkładzie dyskretnym. Wartością oczekiwaną (ew. średnią) $X$ nazywamy wartość sumy  

$EX = \sum_{x \in \mathbb{R}} x P(X=x)$,  

o ile jest ona absolutnie zbieżna.

**Przykład 5.2**  

Założenie absolutnej zbieżności jest niejako konieczne - nie chcemy, żeby wartość $EX$ zależała od kolejności sumowania. Z drugiej strony prowadzi ono czasem do zaskakujących wyników. Rozważmy zmienną $X$ zdefiniowaną następująco: $X$ przyjmuje tylko wartości postaci $2^k$ i $-2^k$ dla $k \ge 2$, przy czym $P(X=2^k) = P(X=2^{-k}) = \frac{1}{2^k}$. Zmienna ta ma rozkład symetryczny względem $0$, intuicyjnie więc jej wartością oczekiwaną powinno być $0$. Ponieważ jednak szereg definiujący $EX$ nie jest absolutnie zbieżny (składa się on z nieskończenie wielu wartości $1$ i nieskończenie wielu $-1$), to $EX$ jest nieokreślone.

**Przykład 5.3**  

Spróbujmy obliczyć wprost z definicji wartość oczekiwaną zmiennej o rozkładzie Bernoulliego i zmiennej o rozkładzie dwumianowym.

Dla $X$ o rozkładzie Bernoulliego z prawdopodobieństwem sukcesu $p$ mamy  

$EX = 0 \cdot P(X=0) + 1 \cdot P(X=1) = P(X=1) = p$.  

Dla $Y$ o rozkładzie dwumianowym $Binom(n,p)$ mamy  

$EY = \sum_{k=0}^n k P(X=k) = \sum_{k=0}^n k {n \choose k}p^k(1-p)^{n-k} = n \sum_{k=1}^n \frac{k}{n}{n \choose k}p^k(1-p)^{n-k}$.  

Korzystając z pochłaniania dostajemy  

$n \sum_{k=1}^n {n-1 \choose k-1}p^k(1-p)^{n-k} = np \sum_{k=0}^{n-1} {n-1 \choose k}p^k(1-p)^{n-1-k} = np$.  

(bo ostatnia suma jest po prostu rozwinięciem dwumianu $(p+(1-p))^{n-1}$).

Jest to bardzo interesujący wynik. Zmienna $Y$ jest sumą $n$ zmiennych $Y_1,\ldots,Y_n$, gdzie każda ze zmiennych $Y_i$ ma rozkład Bernoulliego z prawdopodobieństwem sukcesu $p$. Okazuje się, że $EY = \sum_{i=1}^n EY_i$. Czyżby wartość oczekiwana była addytywna, a nasze rozwlekłe obliczenia $EY$ zupełnie niepotrzebne? Już wkrótce poznamy odpowiedź na to pytanie.

---

## Własności wartości oczekiwanej

Bardzo przydatną i fundamentalną własność wartości oczekiwanej opisuje poniższe twierdzenie  

**Twierdzenie 5.4**  

Niech $X:\Omega \rightarrow \mathbb{R}$ będzie dyskretną zmienną losową o skończonej wartości oczekiwanej. Niech ponadto $\Omega$ będzie  

przeliczalna, lub ogólniej, niech $\sum_{\omega \in \Omega} P(\omega) = 1$. Wtedy:  

$EX = \sum_{\omega \in \Omega} P(\omega) X(\omega)$.

Innymi słowy: zamiast sumować po możliwych wartościach zmiennej $X$ możemy sumować po zdarzeniach elementarnych.

**Dowód**  

$EX = \sum_{x \in \mathbb{R}} x P(X=x) = \sum_{x \in \mathbb{R}} x \sum_{\omega \in \Omega} P(\omega)[X(\omega) = x] = \sum_{\omega \in \Omega} P(\omega) \sum_{x \in \mathbb{R}}x [X(\omega) = x] = \sum_{\omega \in \Omega} P(\omega) X(\omega)$.

Z twierdzenia 5.4 w prosty sposób wynika następujący:

**Wniosek 5.5**  

Jeśli $X$ jest zmienną losową o rozkładzie dyskretnym, a $f:\mathbb{R}\rightarrow \mathbb{R}$ dowolną funkcją, to zachodzi  

$Ef(X) = \sum_{x \in \mathbb{R}} P(X=x) f(x)$,  

o ile $Ef(X)$ istnieje.  

**Dowód**  

Wystarczy popatrzeć na $f$ jako na zmienną losową określoną na $\mathbb{R},P_X$, ew. powtórzyć dowód twierdzenia 5.4.

Wniosek ten okaże się bardzo przydatny przy obliczaniu wariancji - pojęcia, które wkrótce zdefiniujemy.

Twierdzenie 5.4 ma zaskakująco wiele zastosowań i warto o nim pamiętać, nawet jeśli wydaje się zupełnie oczywiste (a może właśnie szczególnie wtedy). Zobaczmy przykład:

**Przykład 5.6**  

Spróbujmy obliczyć wartość oczekiwaną sumy oczek w rzucie dwiema kostkami. Niech $X$ będzie zmienną opisującą sumę oczek. Wtedy z definicji $EX$ mamy  

$EX = \sum_{k=2}^{12} kP(X=k)$.  

Należałoby teraz obliczyć wszystkie wartości $P(X=k)$. Nie jest to bardzo trudne, ale jest nieco uciążliwe i łatwo się przy tych obliczeniach pomylić.

Spróbujmy inaczej. Przyjmijmy, że $\Omega = \{ (i,j) : 1 \le i,j \le 6\}$ i oczywiście $P((i,j)) = \frac{1}{36}$ dla każdych $1 \le i,j \le 6$. Wtedy $X((i,j)) = i+j$ i z twierdzenia 5.4 mamy  

$EX = \sum_{1 \le i,j \le 6} \frac{1}{36} (i+j)$.  

Oczywiście $\sum_{1 \le i,j \le 6} i = \sum_{1 \le i,j \le 6} j$ z symetrii, więc  

$EX = \frac{1}{36} \cdot 2 \cdot \ \sum_{1 \le i,j \le 6} i = \frac{1}{18} \cdot 6 \cdot \sum_{1 \le i \le 6} i = \frac{1}{3} \cdot 21 = 7$.  

Wyprowadzenie wymagające więcej spostrzegawczości niż rachunków, zdecydowanie mniej uciążliwe niż nasz pierwszy pomysł.

Twierdzenie 5.4 pozwala też w prosty sposób pokazać zapowiadaną wcześniej addytywność wartości oczekiwanej (choć nie w pełnej ogólności):  

**Twierdzenie 5.7(Liniowość wartości oczekiwanej)**  

Niech $X,Y$ dyskretne zmienne losowe o skończonej wartości oczekiwanej. Wtedy:

1. $E(cX) = cEX$,
2. $E(X+Y) = EX+EY$.

**Dowód**  

Jeśli $\sum_{\omega \in \Omega} P(\omega) = 1$ (np. $\Omega$ jest przeliczalna), to pierwszy punkt tezy natychmiast wynika z twierdzenia 5.4:  

$E(cX) = \sum_{\omega \in \Omega} P(\omega) cX(\omega) = c\sum_{\omega \in \Omega} P(\omega) X(\omega) = cEX$.  

Drugi punkt nie jest dużo trudniejszy:  

$E(X+Y) = \sum_{\omega \in \Omega} P(\omega) (X+Y)(\omega) = \sum_{\omega \in \Omega} P(\omega) X(\omega) + \sum_{\omega \in \Omega} P(\omega) Y(\omega) = EX+EY$.  

Ogólny przypadek nie jest dużo trudniejszy. Zbiory postaci $X=x\wedge Y=y = \{\omega \in \Omega: X=x,Y=y\}$ stanowią podział $\Omega$ i zachodzi  

$\sum_{x,y \in \mathbb{R}} P(X=x \wedge Y=y) = 1$  

bo $X$ i $Y$ dyskretne. Można zatem myśleć o tych zbiorach jako o elementach pewnej nowej przestrzeni probabilistycznej, na której są określone $X$ i $Y$ i która spełnia założenia twierdzenia 5.4. Bardziej formalnie możemy nasz dowód uogólnić następująco:  

$E(X+Y) = \sum_{z \in \mathbb{R}} P(X+Y=z) z = \sum_{z \in \mathbb{R}} \sum_{x,y | x+y=z} P(X=x \wedge Y=y) (x+y)=\sum_{x,y \in \mathbb{R}} P(X=x \wedge Y=y) (x+y) =$  

$= \sum_{x,y \in \mathbb{R}} P(X=x\wedge Y=y) x + \sum_{x,y\in\mathbb{R}} P(X=x\wedge Y=y) y = EX+EY$.  

Podobnie uogólniamy pierwszą część dowodu.

Trudno jest przecenić znaczenie tego twierdzenia - jeśli musielibyśmy wskazać w całym kursie rachunku prawdopodobieństwa jedno twierdzenie o największym znaczeniu w informatyce teoretycznej, to prawdopodobnie byłaby nim właśnie liniowość wartości oczekiwanej. Siła tego twierdzenia bierze się przede wszystkim stąd, że nie wymaga ono żadnych założeń, w szczególności zmienne $X$ i $Y$ nie muszą być niezależne.

**Przykład 5.8**  

Spróbujmy raz jeszcze obliczyć oczekiwaną sumę oczek z dwóch kostek. Tym razem przedstawimy sumę oczek $X$ jako $X = X_1+X_2$, gdzie $X_1,X_2$ są wynikami z poszczególnych kostek. Wtedy  

$EX = EX_1+EX_2 = 2\sum_{i=1}^6 \frac{1}{6} i = 7$.

**Przykład 5.9**  

Wrzucamy losowo $n$ kul do $n$ urn. Jaka jest wartość oczekiwana frakcji pustych urn?

Niech $X_i$ będzie zmienną, która przyjmuje wartość $1$ jeśli $i$-ta urna jest pusta, a wartość $0$ gdy nie jest pusta. Wtedy $X=X_1+\ldots+X_n$ jest liczbą pustych urn. Mamy  

$EX_i = P(X_i=1) = (1-\frac{1}{n})^n.$  

A zatem z liniowości dostajemy  

$EX = EX_1+\ldots+EX_n = n(1-\frac{1}{n})^n$.  

A zatem oczekiwana frakcja pustych urn jest równa  

$(1-\frac{1}{n})^n$.  

Co ciekawe dla $n \rightarrow \infty$ wartość ta zbiega do $\frac{1}{e}$.

Ten przykład pokazuje siłę twierdzenia o liniowości wartości oczekiwanej. Zachęcamy czytelnika do próby rozwiązania powyższego zadania wprost z definicji.

Skoro $E(X+Y) = EX+EY$, to naturalne wydaje się pytanie, czy zachodzi $E(XY) = EXEY$, czyli czy wartość oczekiwana jest multiplikatywna. Łatwo zauważyć, że nie może to być prawdą - wystarczy wziąć $X$ o rozkładzie Bernoulliego i $Y=X$.

Okazuje się jednak, że czasem wartość oczekiwana jest multiplikatywna:  

**Twierdzenie  5.10**  

Jeśli $X,Y$ niezależne zmienne dyskretne o skończonych wartościach oczekiwanych, to $E(XY) = EXEY$.  

**Dowód**  

$E(XY) = \sum_{z \in \mathbb{R}} P(XY = z) z = \sum_{z \in \mathbb{R}} \sum_{x \in \mathbb{R}\setminus \{0\}} z P(X = x \wedge Y = \frac{z}{x}) = \sum_{z \in \mathbb{R}} \sum_{x \in \mathbb{R}\setminus \{0\}} xP(X = x)\frac{z}{x}P(Y = \frac{z}{x})$.  

Zmieniając kolejność sumowania i podstawiając $y = \frac{z}{x}$ dostajemy  

$E(XY) = \sum_{x \in \mathbb{R}\setminus \{0\}} xP(X = x) \sum_{y \in \mathbb{R}} yP(Y = y) = EXEY$.

Na koniec odnotujmy bardzo przydatny wzór na wartość oczekiwaną zmiennej o wartościach naturalnych:  

**Twierdzenie 5.11**  

Niech $X$ będzie zmienną losową o wartościach naturalnych. Wtedy $EX = \sum_{i =1}^\infty P(X \ge i)$.  

**Dowód**  

$EX = \sum_{i=1}^\infty iP(X=i) = \sum_{i=1}^\infty \sum_{j=1}^i P(X=i) = \sum_{j=1}^\infty \sum_{i=j}^\infty P(X=i) = \sum_{j=1}^\infty P(X \ge j)$.  

**Przykład**  

Obliczmy wartość oczekiwaną zmiennej o rozkładzie geometrycznym. Niech $X \sim Geom(p)$. Wtedy $P(X \ge i) = (1-p)^{i-1}$ i z powyższego twierdzenia dostajemy  

$EX = \sum_{i \ge 1} P(X \ge i) = \sum_{i \ge 1} (1-p)^{i-1} = \frac{1}{1-(1-p)} = \frac{1}{p}$.  

Obliczanie $EX$ wprost jest istotnie bardziej skomplikowane - sprowadza się, de facto, do powtórzenia dowodu twierdzenia 5.11.

---

## Warunkowa wartość oczekiwana

W poprzednim wykładzie zdefiniowaliśmy, dla dowolnej dyskretnej zmiennej losowej $X$ i zdarzenia $A$ o niezerowym prawdopodobieństwie, nową zmienną $X|A$.  

Można oczywiście obliczyć wartość oczekiwaną tak zdefiniowanej zmiennej:  

$E(X|A) = \sum_{x \in \mathbb{R}} xP((X|A) = x) = \sum_{x \in \mathbb{R}} xP(X=x|A)$.

Związek między tak określonymi *warunkowymi wartościami oczekiwanymi*, a zwykłą wartością oczekiwaną, jest taki sam jak między prawdopodobieństwami warunkowymi, a zwykłym prawdopodobieństwem:  

**Twierdzenie 5.12 (Wzór na całkowitą wartość oczekiwaną)**  

Niech $X:\Omega\rightarrow\mathbb{R}$ będzie dyskretną zmienną losową i niech $A_1,A_2,\ldots$ będzie podziałem $\Omega$. Wtedy:  

$EX = \sum_{k=1}^\infty P(A_k) E(X|A_k)$.  

**Dowód**  

Na mocy twierdzenia o prawdopodobieństwie całkowitym  

$P(X=x) = \sum_{k=1}^\infty P(A_k) P(X=x|A_k)$  

dla każdego $k \in \mathbb{N}$ i $x \in \mathbb{R}$. Mnożąc tę tożsamość stronami przez $x$ i sumując po wszystkich $x$ dostajemy tezę:  

$EX = \sum_{x \in \mathbb{R}} \sum_{k=1}^\infty xP(A_k) P(X=x|A_k) =  \sum_{k=1}^\infty \sum_{x \in \mathbb{R}} xP(A_k) P(X=x|A_k) = \sum_{k=1}^\infty P(A_k) E(X|A_k)$.

**Uwaga**  

Podobnie jak w przypadku wzoru na prawdopodobieństwo całkowite, prawdziwa jest także wersja powyższego twierdzenia dla skończonych podziałów $\Omega$, dowód analogiczny. Ponadto tak jak w przypadku wzoru na prawdopodobieństwo całkowite, można powyższe twierdzenie traktować jako przepis na obliczanie wartości oczekiwanej przez przypadki.

**Przykład 5.13**  

Korzystając ze wzoru na całkowitą wartość oczekiwaną obliczymy ponownie wartość oczekiwaną zmiennej $X \sim Geom(p)$.  

$EX = P(X = 1) E(X|X=1) + P(X>1) E(X|X > 1) = p \cdot 1 + (1-p) E(X|X > 1)$.  

Zauważmy, że $X|(X > 1)$ ma taki sam rozkład jak $1+X$. Intuicyjnie jest to dość oczywiste, (prosty) formalny dowód dużo ogólniejszego faktu pojawi się na ćwiczeniach.  

A zatem  

$EX = p + (1-p) E(1+X) = 1 + (1-p)EX$.  

Stąd $pEX = 1$ i ostatecznie $EX = \frac{1}{p}$.

---

## Wariancja - motywacja i definicja

Wartość oczekiwana niesie bardzo istotną informację na temat zmiennej losowej. Tym niemniej, ograniczanie się w analizie do samej wartości oczekiwanej może być zwodnicze, a czasem wręcz niebezpieczne.

Jest duża różnica między inwestycją, w której z prawdopodobieństwem $\frac{1}{2}$ zyskujemy $1,000,000$ zł i z prawdopodobieństwem $\frac{1}{2}$ tracimy $800,000$ zł, a inwestycją w której z prawdopodobieństwem $\frac{1}{2}$ zyskujemy $101,000$ i z prawdopodobieństwem $\frac{1}{2}$ zyskujemy $99,000$. W obu przypadkach wartość oczekiwana zysku wynosi $100,000$ zł, a pomimo to większość osób bez wahania wybrałaby drugą opcję.

Podobnie, jest duża różnica między algorytmem, którego oczekiwany czas działania jest równy $cn\log n$, ale który często działa w czasie bliskim zeru i często działa wielokrotnie wolniej niż średnio, a algorytmem o tym samym średnim czasie działania, który prawie zawsze działa w czasie bliskim średniej. Znów jasne jest, że opcja druga jest z reguły bardziej pożądana.

Aby móc porównywać inwestycje w pierwszym przykładzie i algorytmy w drugim, wprowadzimy miarę tego jak bardzo zmienna losowa odchyla się od swojej wartości średniej. Naturalnym pomysłem byłoby rozważenie wielkości $E|X-EX|$. Pomysł ten jest dobry, a tak zdefiniowaną wielkość nazywa się z reguły *średnim odchyleniem $X$*. Posługiwanie się odchyleniem średnim jest jednak z wielu różnych względów dość problematyczne. W dużym uproszczeniu "pojęcie to nie ma dobrych własności", choćby dlatego, że użyta w definicji wartość bezwględna skutecznie utrudnia korzystanie z narzędzi analitycznych takich jak różniczkowanie.

Zamiast średniego odchylenia będziemy używać pojęć wariancji i odchylenia standardowego:  

**Definicja (Wariancja i odchylenie standardowe)**  

*Wariancją* dyskretnej zmiennej losowej $X$ nazywamy wartość  

$VarX = E(X-EX)^2$,  

o ile ona istnieje.  

*Odchyleniem standardowym $X$* nazywamy $\sigma(X) = \sqrt{VarX}$.

W tym miejscu należy się wyjaśnienie kwestii: po co nam aż dwie wielkości?

Wariancja, w przeciwieństwie do średniego odchylenia, ma bardzo dobre własności i pojawia się w wielu sytuacjach w naturalny sposób. Wbrew pozorom nie jest ona jednak dobrym substytutem średniego odchylenia z tego prostego powodu, że średniego odchylenia nie mierzy. Łatwo to zauważyć, jeśli zastanowimy się co się dzieje z wariancją, jeśli pomnożymy zmienną losową przez stałą:  

$Var(cX) = E(cX-E(cX))^2 = E(c(X-EX))^2= c^2E(X-EX)^ = c^2VarX.$  

To nie wygląda dobrze - sensowna miara średniego odchylenia powinna w takiej sytuacji wzrastać $|c|$-krotnie. Rozwiązaniem tego problemu jest odchylenie standardowe, dla którego jak łatwo zauważyć mamy $\sigma(cX) = |c|\sigma(X)$.

Okazuje się, że odchylenie standardowe jest bardzo dobrą miarą "typowych odchyleń" od średniej, w szczególności ma z reguły wartość bardzo bliską odchyleniu średniemu.

---

## Własności wariancji

Wariancję rzadko oblicza się wprost z definicji. Jedną z przydatniejszych metod jest poniższy wzór:  

**Twierdzenie 5.14**  

$VarX = E(X^2) - (EX)^2$.  

**Dowód**  

$VarX = E(X-EX)^2 = E(X^2 - 2XEX + (EX)^2) = E(X^2) - 2(EX)^2 + (EX)^2 = E(X^2) - (EX)^2$.

**Przykład 5.15**  

Spróbujmy za pomocą tego wzoru obliczyć wariancję rozkładu Bernoulliego i rozkładu dwumianowego.

Dla zmiennej $X$ o rozkładzie Bernoulliego z prawdopodobieństwem sukcesu $p$ mamy:  

$VarX = E(X^2) - (EX)^2 = EX-(EX)^2 = p - p^2 = pq$.

Dla zmiennej $Y \sim Binom(n,p)$ mamy:  

$VarY = E(Y^2) - (EY)^2 = \sum_{k=0}^n k^2{n \choose k}p^k(1-p)^{n-k} - (np)^2 = n\sum_{k=1}^n k\frac{k}{n}{n \choose k}p^k(1-p)^{n-k} - (np)^2$.  

Korzystając z pochłaniania dostajemy:  

$n\sum_{k=1}^n k {n-1 \choose k-1}p^k(1-p)^{n-k} - (np)^2 = np \sum_{k=0}^{n-1} (k+1) {n-1 \choose k}p^k(1-p)^{n-1-k} - (np)^2 = np (\sum_{k=0}^{n-1} k{n-1 \choose k}p^k(1-p)^{n-k} + \sum_{k=0}^{n-1} {n-1 \choose k}p^k(1-p)^{n-k}) - (np)^2$.  

Jedno z wyrażeń w nawiasie jest dwumianem $(p+q)^n$, a drugie wartością oczekiwaną zmiennej o rozkładzie $Binom(n-1,p)$. Dostajemy więc:  

$np ((n-1)p + 1) - (np)^2 = np(np+q) - (np)^2 = (np)^2 + npq - (np)^2 = npq$.

Okazało się, że $VarY = nVarX$, co sugeruje, że być może wariancja jest addytywna, tak jak wartość oczekiwana (liniowa być nie może, bo $Var(cX) = c^2VarX$ ). Sprawdźmy:  

$Var(X+Y) = E((X+Y)-E(X+Y))^2 = E((X-EX)+(Y-EY))^2 = E( (X-EX)^2 +2(X-EX)(Y-EY) + (Y-EY)^2) = VarX+VarY + 2E(X-EX)(Y-EY)$.

Prawie się udało, niestety pojawił się dodatkowy człon $E(X-EX)(Y-EY)$, sprawdźmy czy jest on równy 0:  

$E(X-EX)(Y-EY) = E( XY - XEY - YEX + EXEY) = E(XY) - EXEY - EXEY + EXEY  = E(XY) - EXEY$.

I wszystko jasne: wariancja jest addytywna wtw, gdy wartość oczekiwana jest multiplikatywna. Ważny szczególny przypadek takiej sytuacji opisuje poniższe twierdzenie:  

**Twierdzenie 5.16**  

Jeśli dyskretne zmienne losowe $X$ i $Y$ są niezależne i mają skończoną wariancję, to $Var(X+Y) = VarX+VarY$.  

**Dowód**  

Wynika z wcześniejszych rozważań i multiplikatywności wartości oczekiwanej dla zmiennych niezależnych.

**Uwaga 5.17**  

Człon $E(X-EX)(Y-EY)$ nazywa się *kowariancją X i Y*. Kowariancja jest duża/dodatnia dla zmiennych, które razem przyjmują małe wartości i razem duże, czyli są "w tej samej fazie". Małe/ujemne wartości kowariancji oznaczają zmienne "w przeciwnym fazach".

Czasem jesteśmy zmuszeni obliczyć wariancję sumy zmiennych, które nie są niezależne. Poniższy przykład pokazuję bardzo typową sytuację tego rodzaju i standardowy sposób radzenia sobie z zależnością zmiennych 0/1-kowych.  

**Przykład 5.9 (c.d.)**  

Obliczmy wariancję liczby pustych urn. Korzystając z twierdzenia 5.14 mamy  

$VarX = E(X^2) - (EX)^2$.  

Wartość drugiego członu już znamy, aby obliczyć pierwszy rozbijemy $X^2$ na poszczególne składniki i skorzystamy z liniowości wartości oczekiwanej  

$E(X^2) = E(\sum_{i=1}^n X_i)^2 = E(\sum_{i=1}^n\sum_{j=1}^n X_iX_j) = \sum_{i=1}^n\sum_{j=1}^n E(X_iX_j)$.  

W tej sumie występują dwa rodzaje wyrazów:

- wyrazy postaci $E(X_i^2) = E(X_i) = (1-\frac{1}{n})^n$, oraz
- wyrazy postaci $E(X_iX_j) = P(X_i = 1 \wedge X_j = 1) = (1-\frac{2}{n})^n$.

Tych pierwszych jest $n$, drugich - $n^2-n$.  

A zatem  

$VarX = n(1-\frac{1}{n})^n + (n^2-n) (1-\frac{2}{n})^n - n^2 (1-\frac{1}{n})^{2n}$.

---

## Wyższe momenty

Wartość oczekiwana i wariancja są szczególnymi przypadkami następujących dwóch pojęć:  

**Definicja (Moment)**  

Jeśli $X$ jest zmienną losową, to *$k$-tym momentem*  zmiennej losowej $X$ nazywamy wartość wyrażenia $E(X^k)$, o ile ona istnieje.  

**Definicja (Moment centralny)**  

Jeśli $X$ jest zmienną losową i $EX < \infty$, to *$k$-tym momentem centralnym* zmiennej losowej $X$ nazywamy wartość wyrażenia $E(X-EX)^k$, o ile ona istnieje.  

A zatem wartość oczekiwana jest pierwszym momentem zmiennej, a wariancja drugim momentem centralnym.

Z wyższych momentów korzysta się istotnie rzadziej, niż z $EX$ i $VarX$, mają one jednak swoje miejsce w zastosowaniach. Czytelnikowi polecamy zastanowienie się, co mierzą trzeci, a co czwarty moment centralny?

---

## Funkcje tworzące prawdopodobieństwa

Obliczanie wartości oczekiwanej i wariancji wprost z definicji lub za pomocą jednego z wyprowadzonych przez nas wzorów bywa często uciążliwe i pracochłonne. Poznamy teraz metodę, która pozwala często znacznie uprościć te rachunki.

**Definicja (Funkcja tworząca prawdopodobieństwa)**  

Niech $X$ będzie zmienną losową o wartościach naturalnych. Funkcją tworzącą prawdopodobieństwa zmiennej $X$ zmiennej $X$ nazywamy:  

$g_X(t) = \sum_{k=0}^\infty P(X=k) t^k$.

Tak jak to z reguły bywa z funkcjami tworzącymi, często wygodnie jest je traktować jako szeregi formalne i nie przejmować się zbieżnością. Tym niemniej, z twierdzenia o zbieżności zmajoryzowanej wynika łatwo natychmiast następujący:  

**Fakt 5.18**  

Szereg definiujący $g_X(t)$ jest zawsze absolutnie zbieżny co najmniej na przedziale $[-1,1]$.

Czasem wygodniej jest korzystać z następującej tożsamości:  

**Fakt 5.19**  

Dla tych $t$ dla których szereg definiujący $g_X(t)$ jest absolutnie zbieżny zachodzi:  

$g_X(t) = E(t^X)$.  

**Dowód**  

Oczywisty.

Jak obliczać wartość oczekiwaną i wariancję za pomocą funkcji tworzących prawdopodobieństwa? Wystarczy je zróżniczkować.

**(Prawie prawdziwe) twierdzenie**  

Jeśli $X$ o wartościach naturalnych ma skończoną wartość oczekiwaną, to:  

$EX = g_X'(1)$.

**(Prawie poprawny) dowód**  

$g_X'(t) = \sum_{k=0}^\infty kP(X=k) t^{k-1}$.  

Podstawiając $t=1$ dostajemy tezę.

Powyższe rozumowanie wygląda przekonująco, formalnie jednak nie jest całkiem poprawne. Z tego, że szereg $g_X(t)$ jest zbieżny w przedziale $[-1,1]$ wynika, że możemy go zróżniczkować wewnątrz tego przedziału, ale niekoniecznie w $t=1$. Dlatego należy sformułować twierdzenie tak:  

**Twierdzenie 5.20**  

Jeśli $X$ o wartościach naturalnych ma skończoną wartość oczekiwaną, to:  

$EX = \lim_{t\rightarrow 1^-} g_X'(t)$.  

**Dowód**  

Z twierdzenia Abela suma szeregu potęgowego jest funkcją ciągłą (ew. jednostronnie ciągłą) wszedzie tam gdzie jest zbieżna. A zatem:  

$EX =  \sum_{k=0}^\infty kP(X=k) 1^{k-1} =  \lim_{t\rightarrow 1^-} (\sum_{k=0}^\infty kP(X=k) t^{k-1}) = \lim_{t\rightarrow 1^-} g_X'(t)$.

W praktyce tego rodzaju niuansy nie mają znaczenia. Funkcje tworzące prawdopodobieństwa, z którymi będziemy mieli do czynienia będą zbieżne na całej prostej rzeczywistej i problemy opisane powyżej nie będą występować. W szczególności prawdziwy będzie wzór $EX = g_X'(1)$.

Łatwo zgadnąć jak za pomocą funkcji tworzących prawdopodobieństwa oblicza się wariancję. Różniczkując dwukrotnie! Spróbujmy:  

$g_X''(t) = \sum_{k=0}^\infty k(k-1)P(X=k) t^{k-2}$.  

A zatem  

$g_X''(1) = E(X(X-1))$  

oraz  

$g_X''(1) + g_X'(1) = E(X^2)$,  

o ile oczywiście te pochodne istnieją. Z tego właśnie wzoru będziemy korzystać w praktyce, w ogólnym przypadku zachodzi  

**Twierdzenie 5.21**  

Jeśli $X$ o wartościach naturalnych ma skończoną wartość oczekiwaną i wariancję, to  

$E(X^2) = \lim_{t\rightarrow 1^-} (g_X'(t)+g_X''(t))$  

oraz  

$VarX = \lim_{t\rightarrow 1^-} (g_X'(t)+g_X''(t) -(g_X'(t))^2)$.  

**Dowód**  

Analogiczny jak dla twierdzenia 5.20.

**Przykład 5.22**  

Znajdźmy funkcję tworzącą prawdopodobieństwa rozkładu dwumianowego (w tym przypadku mamy do czynienia z wielomianem tworzącym i większość rozważań powyżej mocno się upraszcza). Niech $X \sim Binom(n,p)$. Wtedy  

$g_X(t) = \sum_{k=0}^n {n \choose k}p^kq^{n-k} t^k = (q+pt)^n$.  

Obliczmy pierwszą i drugą pochodną $g_X(t)$:  

$g_X'(t) = np(q+pt)^{n-1}$,  

$g_X''(t) = n(n-1)p^2(q+pt)^{n-2}$.  

A zatem  

$EX = g_X'(1) = np(q+p)^{n-1} = np$, oraz  

$VarX = g_X''(1) + g_X'(1) - (g_X'(1))^2 = n(n-1)p^2 + np - (np)^2 = np-np^2 = npq.$

Udowodnimy teraz kilka własności funkcji tworzących prawdopodobieństwa, które znacząco ułatwiają posługiwanie się nimi.

**Twierdzenie 5.23**  

Niech $X,Y$ będą niezależnymi zmiennymi o wartościach naturalnych. Wtedy  

$g_{X+Y}(t) = g_X(t) g_Y(t)$.  

**Dowód**  

Dla $t \in [-1,1]$ zachodzi  

$g_{X+Y}(t) = E(t^{X+Y}) = E(t^Xt^Y) = E(t^X)E(t^Y) = g_X(t) g_Y(t)$,  

więc musi też zachodzić teza.

Twierdzenie to w naturalny sposób uogólnia się na dowolną skończoną liczbę niezależnych zmiennych losowych.

**Przykład 5.24**  

Korzystając z powyższego twierdzenia możemy policzyć $g_X(t)$ dla $X \sim Binom(n,p)$ w alternatywny sposób. Zauważmy mianowicie, że dla zmiennej $Y$ o rozkładzie Bernoulliego z prawdopodobieństwem sukcesu $p$ mamy  

$g_Y(t) = q+pt$,  

a ponieważ $X$ jest sumą $n$ niezależnych zmiennych o takim właśnie rozkładzie, to  

$g_X(t) = (g_Y(t))^n = (q+pt)^n$.

**Twierdzenie 5.25**  

Jeśli $X$ jest zmienną losową o wartościach naturalnych, a $c \in \mathbb{N}$, to  

$g_{cX}(t) = g_X(t^c)$.  

**Dowód**  

$g_{cX}(t) = \sum_{k=0}^\infty P(cX = k) t^k = \sum_{i=0}^\infty P(cX = ci) t^{ci} = \sum_{i=0}^\infty P(X = i) (t^c)^i = g_X(t^c)$.

**Twierdzenie 5.26**  

Niech $N,X_1,X_2,\ldots,X_N$ będą niezależnymi zmiennymi losowymi o wartościach naturalnych. Ponadto, niech wszystkie $X_i$ mają ten sam rozkład $X_i \sim X$. Wtedy dla $S = X_1 + \ldots + X_N$ zachodzi  

$g_S(t) = g_N(g_X(t))$  

oraz  

$ES = ENEX$.  

**Dowód**  

Dla $t \in [-1,1]$ mamy z twierdzenia o całkowitej wartości oczekiwanej  

$g_S(t) = E(t^S) = \sum_{k=0}^\infty P(N=k) E(t^S | N=k) = \sum_{k=0}^\infty P(N=k) E(t^{X_1+\ldots+X_k})$.  

Z niezalezności $X_i$ dostajemy  

$g_S(t) = \sum_{k=0}^\infty P(N=k) E(t^{X_1})\cdot \ldots \cdot E(t^{X_k}) = \sum_{k=0}^\infty P(N=k) (g_X(t))^k = g_N(g_X(t))$.  

Druga część twierdzenia wynika z pierwszej, ciągłości funkcji tworzących w $t=1$ oraz tego, że $g_X(1) = 1$.  

$ES = \lim_{t \rightarrow 1^-} g_S'(t) = \lim_{t \rightarrow 1^-} g_N'(g_X(t))\cdot g_X'(t) = \lim_{t \rightarrow 1^-} g_N'(t) g_X'(t) = ENEX$.

**Przykład 5.27**  

Rzucamy kostką, niech $N$ będzie wynikiem rzutu. Następnie rzucamy $N$ razy monetą. Jaki rozkład ma łączna liczba orłów? W szczególności jak wygląda funkcja tworząca prawdopodobieństwa tego rozkładu?

Niech $X_1,X_2,\ldots,$ będą wynikami rzutów monetą. Wtedy łączna liczba orłów jest równa  

$S = X_1+\ldots+X_N$  

i mamy do czynienia z sytuacją z udowodnionego właśnie twierdzenia.

Ponieważ  

$g_N(t) = \frac{1}{6}\sum_{i=1}^6 t^i$  

oraz  

$g_{X_i}(t) = \frac{1+t}{2}$,  

to  

$g_S(t) = \frac{1}{6}\sum_{i=1}^6 (\frac{1+t}{2})^i$.
