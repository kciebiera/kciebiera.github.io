---
title: "Wykład 1: Przestrzeń probabilistyczna, schemat klasyczny"
source_url: "http://smurf.mimuw.edu.pl/node/706"
source_kind: "html"
---

# Wykład 1: Przestrzeń probabilistyczna, schemat klasyczny

## Losowość i prawdopodobieństwo

Zacznijmy ten wykład od zastanowienia się nad tym czym jest prawdopodobieństwo pojawiające się w jego tytule, i czym jest losowość, która się z prawdopodobieństwem kojarzy. Wydaje się, że mamy z tymi pojęciami do czynienia całkiem często na co dzień. Oto kilka sytuacji, w których pojawia się pojęcie prawdopodobieństwa/szans:

- Często słyszymy, jak ktoś mówi, że są na coś małe bądź duże szanse, albo że coś jest bardzo lub niezbyt prawdopodobne.
- Eksperci w grach o charakterze losowym takich jak poker, backgammon, czy blackjack doskonale znają prawdopodobieństwa różnych zdarzeń, np. wypadnięcia pary na dwóch kostkach, rozdania dwóch par wśród pięciu kart itp.
- Jeśli zakładam się ze znajomym o wynik meczu w stosunku 1:1, to zapewne sądzę, że prawdopodobieństwo wystąpienia mojego wyniku jest równe co najmniej $50\%$ (ew. jestem zagorzałym fanem i muszę bronić honoru swojej drużyny).
- Przedstawiciel instytutu badania opinii publicznej może tłumaczyć, że wynik przeprowadzonego przez niego sondażu z prawdopodobieństwem $95\%$ przybliża aktualne poparcie dla danej partii z błędem $\pm 2\%$.
- Lekarz może twierdzić, że szanse powodzenia operacji, którą będzie przeprowadzał wynoszą $60\%$. Podobnie może się wypowiadać o szansach na powodzenie kuracji, itp.

Zauważmy w tym miejscu, że pojęcie prawdopodobieństwa jest tu używane na bardzo różne sposoby:

- Prawdopodobieństwo "subiektywne": Czasem używamy pojęcia prawdopodobieństwa, aby wyrazić nasze przekonanie co do tego jak potoczy się przyszłość, jaki jest prawdziwy stan rzeczy, co tak naprawdę stało się w przeszłości. Mówimy na przykład "na $70\%$ on już wie", a przecież on albo wie albo nie wie. Nie mówimy więc tu o zjawisku o charakterze losowym, tylko o naszym stopniu pewności co do wypowiadanej opinii.
- Prawdopodobieństwo "statystyczne": Czasem używamy go w tzw. wersji statystycznej. Jeśli wiemy, że w przeszłości na 100 prób pewne zdarzenie zachodziło średnio 43 razy, to wydaje się sensowne zakładać, że tak samo (lub podobnie) będzie w przyszłości. W ten sposób rozumuje lekarz z jednego z naszych przykładów, posiłkując się danymi statystycznymi.
- Prawdopodobieństwo "analityczne": Zdarza się też, że spodziewamy się, że pewien wynik ma dane prawdopodobieństwo nie dlatego, że dysponujemy danymi statystycznymi dotyczącymi analogicznych sytuacji, ale ze względu na jakiś rodzaj symetrii. Wydaje się rozsądne założyć, że prawdopodobieństwo wypadnięcia reszki na monecie jest takie samo jak orła. Podobnie jeśli grają dwie drużyny, o których nic nie wiemy, to z naszego punktu widzenia mają one równe szanse na zwycięstwo.

Pojęcie losowości jest równie skomplikowane jak pojęcie prawdopodobieństwa:

- Zakładamy, że wynik rzutu monetą jest losowy. Podobnie możemy mówić, że losowe są wyniki LOTTO, czy nawet wynik meczu piłkarskiego.
- Często słyszymy sformułowania "Ależ to zupełna loteria" albo "Równie dobrze możemy rzucić monetą" itp, sugerujące, że autor wypowiedzi uważa pewną sytuację za mającą w dużym stopniu charakter losowy.
- W większości języków programowania dostępne są funkcje zwracający liczby losowe. Często nazywane są one pseudolosowymi i słusznie, bo wcale losowe nie są, ale w miejscach gdzie losowość jest bardzo ważna, projektuje się generatory liczb losowych, które korzystają z różnego rodzaju zjawisk fizycznych, i wtedy uważa się liczby przez nie generowane za losowe.

Ale co to właściwie znaczy, że coś jest losowe? Że nie da się tego przewidzieć? A nie da się przewidzieć wyniku rzutu monetą? Może się da... A wynik meczu? Kto wie, może problem leży w naszej niewystarczającej mocy obliczeniowej i nieumiejętności wystarczającego dokładnego uwzględnienia wszystkich parametrów.

Z powyższych rozważań wynika, że pojęcia prawdopodobieństwa i losowości, tak jak są one rozumiane i używane w języku codziennym, są dość niejasne i trudno byłoby uprawiać ścisłą naukę za ich pomocą. Dlatego nie będziemy tego robić. Zamiast tego zdefiniujemy w bardzo skrupulatny sposób pewną klasę obiektów matematycznych, formalnych modeli zjawisk o charakterze losowym, tzw. przestrzenie probabilistyczne. O przestrzeniach probabilistycznych można rozumować w sposób zupełnie ścisły, a problemy, o których wspomnieliśmy na początku wykładu zaczynają się pojawiać dopiero wtedy, gdy pytamy o to jaka przestrzeń probabilistyczna najlepiej opisuje interesującą nas sytuację.

Warto w tym miejscu zwrócić uwagę na jeszcze jedną rzecz. Jak pokazała nasza wstępna dyskusja z pojęciami szans i losowości spotykamy się bardzo często na co dzień. Dlatego, o ile napotkany na ulicy przechodzień prawdopodobnie nie będzie się czuł szczególnie mocno w teorii przestrzeni Banacha, czy nawet całkowaniu, o tyle prawdopodobnie będzie miał bardzo wiele do powiedzenia na temat tego jak prawdopodobne są różne wyniki opisanego przez nas doświadczenia o charakterze losowym. Czyli wszyscy jesteśmy "ekspertami od prawdopodobieństwa". Taka sytuacja jest oczywiście w jakimś sensie dobra - nie musimy się uczyć wszystkiego od początku - ale też może być bardzo niebezpieczna. Mętna i pozbawiona solidnych podstaw wiedza jest często dużo gorsza niż brak jakiejkolwiek wiedzy. Dlatego zachęcam Państwa do świeżego spojrzenia na to, co będzie się działo w kolejnych wykładach i, w miarę możliwości, oderwania się od wszelkiego rodzaju intuicji i "wiedzy" ze szkoły średniej, przynajmniej przez jakiś czas. Formalizm, który tu zdefiniujemy dość długo będzie sprawiał wrażenie zbędnego obciążenia, aż do momentu kiedy nagle okaże się niezbędny, co może się okazać bardzo nieprzyjemną niespodzianką dla kogoś, kto się do niego nie przyzwyczai.

---

## Przestrzeń probabilistyczna

Zanim zdefiniujemy przestrzeń probabilistyczną wprowadzimy kilka definicji, które w znacznym stopniu ułatwią dalsze rozważania. Za pomocą przestrzeni probabilistycznych będziemy modelować tzw. doświadczenia losowe:  

**Definicja (doświadczenie losowe i zdarzenie elementarne)**  

*Doświadczeniem/eksperymentem losowym* nazywamy dowolny proces (lub ciąg czynności) taki, że:

- jego sposób wykonywania i warunki są ściśle określone, a sam proces można dowolnie wiele razy powtarzać,
- zbiór możliwych wyników procesu, tzw. *zdarzeń elementarnych* jest z góry znany,
- wyniku konkretnego doświadczenia nie można z góry przewidzieć - ma on charakter losowy.

**Uwaga 1.1** Powyższa definicja ma nieco inny charakter niż definicje, z którymi spotykamy się z reguły w teoriach matematycznym. Nie definiujemy tutaj formalnie pojęcia doświadczenia losowego, staramy się raczej wskazać jakie jego cechy będą dla nas istotne, oraz wprowadzić kluczowe terminy.  

W szczególności założenia powtarzalności doświadczenia nie należy traktować zbyt dosłownie. Na przykład rozwiązując zadania o pojedynku, w którym każdy z uczestników z pewnym prawdopodobieństwem ginie, trudno sobie wyobrazić powtarzanie doświadczenia dowolnie wiele razy. W tego typu sytuacjach mamy na myśli raczej pewnego rodzaju eksperyment myślowy, niż faktyczne powtarzanie eksperymentu. Tym niemniej, założenie powtarzalności jest dość istotne jako podstawa intuicji częstościowej, o której powiemy później.

**Definicja (zdarzenie)**  

*Zdarzeniem* nazywamy dowolny podzbiór zbioru zdarzeń elementarnych.

**Przykład 1.2**  

Jeśli rozważanym doświadczeniem losowym jest rzut kostką, to zbiorem zdarzeń elementarnych może być na przykład zbiór $\Omega = \{1,\ldots,6\}$. W tym przypadku zdarzeniami są np. zbiory $A = \{2,4,6\}$,  $B=\{1,2,3,4\}$, $C = \{1,2,3,4,5,6\}$, czy $D = \emptyset$. Każde takie zdarzenie $X \subseteq \Omega$ należy interpretować jako "wynikiem doświadczenia było jedno ze zdarzeń elementarnych w $X$". W szczególności:

- zdarzenie $A$ to "wypadła parzysta liczba oczek",
- $B$ - "wypadły co najwyżej cztery oczka",
- $C$ - "coś wypadło",
- $D$ - "nic nie wypadło".

To ostatnie zdarzenie jest oczywiście niemożliwe (i tak się je właśnie często nazywa), co nie zmienia faktu, że jest ono zdarzeniem.

Naszym modelem eksperymentu losowego będzie przestrzeń probabilistyczna. Przestrzeń probabilistyczna będzie się składać ze zbioru zdarzeń elementarnych oraz z pewnego sposobu przypisania zdarzenim liczb z przedziału $[0,1]$, które będziemy nazywać ich prawdopodobieństwami. Definicja, której użyjemy będzie zaskakująco skomplikowana. Dlatego zanim ją podamy, zobaczmy dlaczego nie działają naturalne, prostsze pomysły.

**Definicja przestrzeni probabilistycznej - pierwsze podejście**  

Najbardziej naturalną i najprostszą metodą definiowania przestrzeni probabilistycznej wydaje się przyjęci, że przestrzeń taka jest parą $(\Omega,P)$, gdzie $P:\Omega \rightarrow [0,1]$ jest funkcją przypisującą prawdpodobieństwo każdemu zdarzeniu elementarnemu. Funkcję taką możemy w naturalny sposób rozszerzyć na dowolne podzbiory $A \subseteq \Omega$ w następujący sposób  

$P(A) = \sum_{a \in A} P(a)$.  

Problemy z tym podejściem pojawią się, gdy spróbujemy wymodelować w ten sposób losowanie punktu z odcinka $[0,1]$. Intuicyjnie, każdy punkt powinien być równie prawdopodobny, ale skoro tak, to prawdopodobieństwa wszystkich punktów muszą być równe $0$, w przeciwnym przypadku dostaniemy $P(\Omega) = \infty$.  

Widać więc, że nie możemy definiować $P$ wychodząc od zdarzeń elementarnych - trzeba $P$ definiować bezpośrednio na podzbiorach $\Omega$.

**Definicja przestrzeni probabilistycznej - drugie podejście**  

Kolejny naturalny pomysł to zdefiniowanie przestrzeni probabilistycznej jako pary $(\Omega,P)$, gdzie $P:2^\Omega \rightarrow \mathbb{R}$. Oczywiście funkcja $P$ pojawiająca się w tej definicji powinna spełniać pewne naturalne własności, np. $P(A \cup B) = P(A)+P(B)$, jeśli $A,B \subseteq \Omega$ są rozłączne. Okazuje się, że takie funkcje $P$ to nic innego jak miary, o których była mowa na wykładzie z analizy matematycznej. I tu pojawia się problem: Jeśli, tak jak poprzednio, spróbujemy zdefiniować przestrzeń probabilistyczną odpowiadającą losowaniu punktu z odcinka, to będziemy musieli określić miarę $P:2^{[0,1]} \rightarrow [0,1]$. Co więcej, aby miara ta odpowiadała naszej intuicji losowania punktu z odcinka powinna ona spełniać pewne dodatkowe własności, np. $P(A) = P(a+A)$ dla dowolnych $a \in [0,1]$ oraz $A,a+A \subseteq [0,1]$, gdzie $a+A = \{a+x : x \in A\}$. Niestety, jak wiemy z wykładu z analizy matematycznej, takiej miary skonstruwać się nie da. Najlepsze co możemy zrobić to zdefiniować miarę na dość dużej rodzinie podzbiorów $[0,1]$, tzw. zbiorach mierzalnych w sensie Lebesgue'a.

Powyższe rozważania prowadzą do następującej definicji przestrzeni probabilistycznej:  

**Definicja (przestrzeń probabilistyczna)**  

Przestrzenią probabilistyczną nazywamy trójkę $(\Omega,\mathcal{F},P)$, gdzie $\mathcal{F} \subseteq 2^\Omega$ oraz $P:\mathcal{F} \rightarrow [0,1]$. Ponadto $\mathcal{F}$ musi spełniać następujące warunki:

1. $\emptyset \in \mathcal{F}$,
2. jeśli $A \in \mathcal{F}$, to $\Omega \setminus A \in \mathcal{F}$ (zamkniętość na dopełnienia),
3. jeśli $A_1,A_2,\ldots \in \mathcal{F}$ to $\bigcup_{i=1}^\infty A_i \in \mathcal{F}$ (zamkniętość na przeliczalne sumy),

a $P$ następujące warunki:

1. $P(\Omega) = 1$,
2. jeśli $A_1,A_2,\ldots \in \mathcal{F}$ są parami rozłączne, to $P(\bigcup_{i=1}^\infty A_i) = \sum_{i=1}^\infty P(A_i)$. (przeliczalna addytywność)

**Uwaga 1.3**  

W kolejnych wykładach bardzo często będziemy mieli do czynienia z sytuacjami, w których $\Omega$ jest zbiorem skończonym, ew. nieskończonym przeliczalnym. W takich sytuacjach problemy z mierzalnością nie mogą się pojawić i w powyższej definicji można zawsze przyjąć $\mathcal{F} = 2^\Omega$. Dla uproszczenia notacji będziemy wtedy zamiast pisać $(\Omega,P)$ zamiast $(\Omega,\mathcal{F},P)$, oraz $A \subseteq \Omega$ zamiast $A \in \mathcal{F}$.

**Uwaga 1.4**  

Dość często będą nas interesowały wartości $P$ na podzbiorach jednoelementowych. Będziemy wtedy dla uproszczenia pisać $P(x)$ zamiast $P(\{x\})$.

Wprost z definicji przestrzeni probabilistycznej prosto wynika wiele naturalnych własności prawdopodobieństwa, oto kilka z nich:  

**Fakt 1.5 (własności prawdopodobieństwa)**  

Niech $(\Omega,\mathcal{F},P)$ będzie przestrzenią probabilistyczną. Wtedy:

1. dla dowolnego $A \in \mathcal{F}$ zachodzi $\bar{A} = \Omega \setminus A \in \mathcal{F}$ i ponadto $P(\bar{A}) = 1 - P(A)$,
2. $P(\emptyset) = 0$,
3. jeśli $A_1,A_2,\ldots \in \mathcal{F}$ to $\bigcap_{i=1}^\infty A_i \in \mathcal{F}$,
4. jeśli $A,B \in \mathcal{F}$ i $A \subseteq B$, to $P(A) \le P(B)$,
5. jeśli $A_1,A_2,\ldots \in \mathcal{F}$, to $P(\bigcup_{i=1}^\infty A_i) \le \sum_{i=1}^\infty P(A_i)$.

**Dowód**  

Z zamkniętości $\mathcal{F}$ na dopełnienia wynika, że $\bar{A} \in \mathcal{F}$. Możemy więc użyć przeliczalnej addytywności $P$ i dostajemy $P(A)+P(\bar{A}) = P(\Omega) =1$, co daje tezę.  

2. wynika z 1. dla $A=\emptyset$.  

3. wynika z zamkniętości $\mathcal{F}$ na przeliczalne sumy i praw de Morgana,  

4. wynika z przeliczalnej addytywności $P$. Mamy bowiem $P(B) = P(A) + P(B \setminus A)$ (to, że $B \setminus A = B \cap \bar{A} \in \mathcal{F}$ wynika z 3.),  

Wreszcie, aby pokazać 5. zdefiniujmy dla $B_i = A_i \setminus \bigcup_{j=1}^{i-1} A_j$ dla $i=1,2,\ldots$. Wtedy mamy $\bigcup_{i=1}^\infty A_i = \bigcup_{i=1}^\infty B_i$ oraz $B_i \subseteq A_i$ i ponadto $B_i$ są parami rozłączne , a zatem:  

$P(\bigcup_{i=1}^\infty A_i) = P(\bigcup_{i=1}^\infty B_i) = \sum_{i=1}^\infty P(B_i) \le \sum_{i=1}^\infty P(A_i)$.

---

## Schemat klasyczny

Zastanowimy się teraz jak powinna wyglądać funkcja $P$ z definicji przestrzeni probabilistycznej. Jest dość jasne, że podane przez nas aksjomaty nie wyznaczają $P$ jednoznacznie. Na przykład, jeśli rzucamy monetą i bierzemy $\Omega = \{O,R\}$, to z aksjomatów wynika jedynie, że  

$P(\emptyset)=0,\ P(\Omega) = 1,\ (P(O)+P(R) = 1$.  

A ile jest równe $P(O)$ czy $P(R)$? Wiele osób uważa, że teoria prawdopodobieństwa w jakiś sposób dowodzi tego, że $P(O)=P(R)=\frac{1}{2}$, czyli w szczególności, że wynika to z aksjomatów. Nic z tych rzeczy, i dobrze, że tak nie jest. Gdyby z aksjomatów wynikało, że $P(O)=\frac{1}{2}$, to nie bylibyśmy w stanie wymodelować oszukanej monety!

No dobrze, ale w takim razie skąd weźmie się wartość $P(O)$? Otóż naszym zadaniem jest, w ramach budowania probabilistycznego modelu rozpatrywanego doświadczenia, dobranie $P$ tak, aby jak najlepiej opisywało to doświadczenie. Dobrym punktem wyjścia do definicji $P$ jest często intuicja częstościowa: Jeśli wykonamy bardzo wiele powtórzeń tego samego eksperymentu, to frakcja powtórzeń, w których zaszło zdarzenie $A$ powinna zbiegać do prawdopodobieństwa $A$.

**Uwaga 1.6**  

Opisanej powyżej intuicji częstościowej można użyć do innej (nieaksjomatycznej) definicji przestrzeni probabilistycznej odpowiadającej doświadczeniu losowemu. Możemy mianowicie każdemu zdarzeniu $A$ przypisać prawdopodobieństwo $P(A)$, które jest granicą frakcji powtórzeń doświadczenia, dla których $A$ zaszło. Takie podejście niesie ze sobą wiele problemów, zaczynając od tego, że nie jest całkiem jasne dlaczego taka granica miałaby w ogóle istnieć (w tym miejscu warto się zastanowić nad tym co się stanie jeśli spróbujemy użyć tego podejścia do modelowania losowania punktu z odcinka). W naszym podejściu intuicja częstościowa prawdopodobieństwa jest używana tylko przy wyborze modelu, natomiast sama definicja przestrzeni probabilistycznej jest całkowicie od niej niezależna.

Co daje intuicja częstościowa w przypadku rzutu monetą? Intuicyjnie wydaje się jasne, że na dłuższą metę orzeł i reszka powinny wypadać równie często, to jest w połowie rzutów. Jeśli prawdopodobieństwa mają odpowiadać częstościom, to powinniśmy przyjąć $P(O)=P(R)=\frac{1}{2}$. Mamy wtedy bowiem $P(O)=P(R)$ oraz $P(O) + P(R) = P(\Omega) = 1$. Podkreślmy jednak raz jeszcze, że jest to tylko założenie, na dodatek prawie zawsze fałszywe w przypadku fizycznych monet - która moneta jest idealna? Oczywiście monety wyimaginowane są idealne, jeśli tylko tego chcemy.

Rozumowanie, którego użyliśmy przy wyborze probabilistycznego modelu dla rzutu monetą jest stosowane bardzo często, w nieco ogólniejszej postaci, i ma nawet swoją nazwę.  

**Fakt 1.7 (Schemat klasyczny)**  

Niech $(\Omega,P)$ będzie skończoną przestrzenią probabilistyczną. Jeśli dla każdych $x,y \in \Omega$ zachodzi $P(x) = P(y)$, to:

- $P(x) = \frac{1}{|\Omega|}$ dla każdego $x \in \Omega$,
- $P(A) = \frac{|A|}{|\Omega|}$ dla każdego $A \subseteq \Omega$.

Oczywisty dowód pomijamy.

Schemat klasyczny ma wiele zastosowań, używając go należy być jednak ostrożnym - schemat klasyczny nie zawsze prowadzi do sensownego modelu.

**Przykład 1.8**  

W XVIII-wiecznej encyklopedii d'Alambert pisze, że prawdopodobieństwo uzyskania co najmniej jednego orła w dwóch rzutach wynosi $\frac{2}{3}$. Dlaczego? Otóż, możliwe są $3$ wyniki:  O (wtedy dalsze rzuty nie są istotne), RO i RR. Z tych wyników tylko ostatni nam nie pasuje. Zgadzamy się z tym rozumowaniem? Raczej nie. Wydaje się jasne, że wyniki O,RO,RR nie powinny być równie prawdopodobne.  

Oczywiście można problem rozważany przez d'Alamberta rozwiązać używając schematu klasycznego. Czy widzisz jak?

**Przykład 1.9**  

Przypuśćmy, że interesuje nas prawdopodobieństwo tego, że w losowej $5$-kartowej ręce pokerowej jest dokładnie jedna para (w szczególności nie ma w niej trójki ani dwóch par). Aby obliczyć to prawdopodobieństwo przyjmijmy, że  $\Omega$ to zbiór wszystkich $5$-elementowych podzbiorów zbioru $52$ kart. Niech $A$ będzie zbiorem wszystkich zbiorów zawierających dokładnie jedną parę. Chcemy obliczyć $P(A)$. Ponieważ każdy $5$-elementowy podzbiór kart powinien być równie prawdopodobny, korzystamy ze schematu klasycznego i dostajemy:  

$P(A) = \frac{|A|}{|\Omega|}$.  

Jasne jest, że  $|\Omega| = {52 \choose 5}$, a jaka jest moc $A$? Zastanówmy się na ile sposobów można wybrać podzbiór zawierający dokładnie jedną parę:

1. na $13$ sposobów wybieramy rangę kart w parze,
2. na ${4 \choose 2} = 6$ sposobów wybieramy kolory tych kart,
3. na ${12 \choose 3}$ sposobów wybieramy rangi pozostałych kart (muszą być różne),
4. na $4^3$ sposobów wybieramy kolory tych kart.

Ostatecznie dostajemy $|A| = 13 \cdot 6 \cdot 4^3 \cdot {12 \choose 3}$ oraz $P(A) = \frac{|A|}{|\Omega|} \approx 0.423$.  

Jak widać użycie schematu klasycznego sprowadziło problem probabilistyczny do problemu kombinatorycznego. Na ćwiczeniach zobaczymy więcej tego rodzaju zadań.

---

## Modele, a rzeczywistość

Na koniec wykładu krótka uwaga dotycząca modelowania rzeczywistych doświadczeń przestrzeniami probabilistycznymi: Pamiętajmy, że są to tylko modele i (prawie zawsze) są one jedynie przybliżeniami rzeczywistości. Mówiliśmy już o tym przy okazji przykładu z rzutem monetą, ale zobaczmy jeszcze jeden nieco ciekawszy przykład. Chcemy zbudować model probabilistyczny, który pozwoli nam obliczyć prawdopodobieństwo tego, że losowo wybrana osoba urodziła się $1$ czerwca. Najprostszy pomysł to przyjąć, że $\Omega$ składa się z $365$ dni roku i każdy jest równie prawdopodobny. Ten model prowadzi do odpowiedzi $\frac{1}{365}$, ale nie jest zbyt dobry. Dlaczego? Zapomnieliśmy o latach przestępnych. Uwzględnienie lat przestępnych jest nieco problematyczne, bo nie dość, że trzeba w tym celu obliczyć jak duża jest frakcja lat przestępnych, to zapewne chcielibyśmy przy tym uwzględnić to ile lat wstecz patrzymy. Nie interesują nas przecież lata przestępne $150$ lat temu!

No dobrze, przypuśćmy, że udało nam się jakoś uwzględnić lata przestępne. Czy to jest dobry model? Na pewno jest niezły, ale nie jest idealny. Wiadomo przecież, że nie każda data urodzin jest równie prawdopodobna. W niektórych miesiącach rodzi się, z różnych przyczyn, dużo więcej dzieci niż w innych. Zachęcamy czytelnika do zastanowienia się co jeszcze nie zostało uwzględnione w tym modelu.

Wniosek z tych rozważań jest taki, że model probabilistyczny zawsze będzie kompromisem między prostotą opisu, a dokładnością odwzorowania rzeczywistości.
