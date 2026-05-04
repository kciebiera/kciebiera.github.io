---
layout: 'default'
title: 'Haszowanie 3: haszowanie liniowe'
page_url: 'http://smurf.mimuw.edu.pl/node/1120'
source_url: 'http://smurf.mimuw.edu.pl/node/1120'
source_kind: 'html'
uses_math: true
render_with_liquid: false
rendered_file: 'lectures/zaawansowane-algorytmy-i-struktury-danych/haszowanie-3-haszowanie-liniowe/rendered.html'
---

# Haszowanie 3: haszowanie liniowe

## Algorytmy operacji słownikowych w haszowaniu liniowym

W tym rozdziale przedstawimy kolejne bardzo naturalne rozwiązanie problemu dynamicznego słownika oparte na haszowaniu. Liczby całkowite ze zbioru $S$ reprezentowanego przez nasz słownik będą przechowywane bezpośrednio w tablicy $T[0..m - 1]$. Rozmiar $T$ będzie większy niż $n=\vert{}S\vert{}$, choć liniowy względem $n$. Zwykle w zastosowaniach przyjmuje się $m=2n$, do udowodnienia asymptotycznych oszacowań na czasy działania operacji wystarczy $m=(1+\varepsilon)\cdot n$, natomiast my dla ustalenia uwagi przyjmiemy $m=3n$. Algorytmy operacji słownikowych będą korzystały z pojedynczej funkcji haszującej $h:U\rightarrow \{0..m-1\}$.

Algorytmy poszczególnych operacji są następujące:

- **insert($x$):** Wstawia $x$ w pierwsze wolne miejsce w ciągu komórek tablicy o kolejnych indeksach $h(x), h(x) + 1, h(x) + 2, h(x) + 3, \ldots$ (oczywiście liczymy modulo $m$).
- **lookup($x$):** Sprawdza komórki $T$ o indeksach $h(x), h(x)+1, h(x)+2,\ldots$, aż do znalezienia $x$ (wtedy zwraca True) lub komórki pustej (wtedy zwraca False).
- **delete($x$):** wyszukuje $x$ tak jak opisano powyżej -- powiedzmy, że $T[p] = x$. Zamiast $x$ wstawiamy znak pusty `NULL`. Nie możemy jednak tak zostawić tablicy $T$, ponieważ w spójnym bloku niepustych komórek zaczynającym się od pozycji $p+1$ może znajdować się element (lub elementy) $y$ taki, że $h(y) \le p$; wówczas wynik operacji lookup($y$) byłby niepoprawny. W tym celu znajdujemy pierwszy taki element $y=T[r]$, wstawiamy go w $T[p]$, i czyścimy jego komórkę, tzn. `T[r] := NULL`. Oczywiście tę operację powtarzamy tak długo, jak występuje opisana powyżej niepożądana sytuacja. Dla jasności poniżej podajemy pseudokod. ``` procedure delete (x) begin p := h(x); while (T[p] <> x) and (T[p] <> NULL) p := p + 1; T[p] := NULL fill(p) end; procedure fill(p) begin r := p+1; while (T[r] <> NULL) and (h(T[r]) > p) r := r + 1; if (T[r] <> NULL) then begin T[p] := T[r]; T[r] := NULL; fill(r); end; end; ```

Poprawność opisanych algorytmów powinna być dość jasna (w zasadzie tylko w przypadku delete jest się nad czym zastanawiać).

Zauważmy, że opisane operacje (szczególnie lookup i insert) operują z reguły na spójnym bloku pamięci (chyba że, pechowo, algorytm wyszukiwania musi ,,przeskoczyć'' z końca tablicy na początek). Taki sposób dostępu do pamięci jest kluczowy dla praktycznej efektywności algorytmów, gdyż w komputerach o współczesnej architekturze z reguły wykonany zostanie odczyt pojedynczego bloku pamięci przy pierwszej operacji dostępu do tablicy T, natomiast wartości kolejnych komórek zostaną już odczytane z szybkiej pamięci typu cache. Z tego względu haszowanie liniowe jest rozwiązaniem najczęściej wybieranym w praktyce. W tym wykładzie dowiemy się, że odpowiednio wybierając funkcję haszującą możemy zagwarantować także teoretyczną efektywność wszystkich operacji słownikowych.

## Analiza oczekiwanej złożoności czasowej operacji

Pamiętamy z wcześniejszego [wykładu](http://smurf.mimuw.edu.pl/drupal6/node/1102), że przy haszowaniu przez łańcuchowanie, jeśli $h$ jest wybrana losowo z rodziny uniwersalnej, to oczekiwane czasy operacji słownikowych są stałe. Od czasu uzyskania tego wyniku przez Cartera i Wegmana w 1977r. pozostawało otwartym pytanie, czy można uzyskać analogiczny wynik dla haszowania liniowego, używając rodziny uniwersalnej lub jakiejś innej rodziny funkcji haszujących. Dopiero w 2007 odpowiedzi udzielili Rasmus Pagh, Anna Pagh i Milan Ruzic.

**Twierdzenie 1**  

Jeśli w haszowaniu liniowym $h$ jest wybrana losowo z uniwersalnej rodziny funkcji haszujących Cartera i Wegmana (patrz [Przykład 2](http://smurf.mimuw.edu.pl/drupal6/node/1102) w I wykładzie o haszowaniu), to istnieje $S\subset U$ taki, że całkowity oczekiwany czas wstawienia wszystkich elementów z $S$  wynosi $\Omega(n\log n)$.

Równocześnie pokazali oni także następujący wynik pozytywny:

**Twierdzenie 2**  

Jeśli w haszowaniu liniowym $h$ jest wybrana losowo z dowolnej 5-niezależnej rodziny funkcji haszujących, to oczekiwany czas operacji słownikowych jest stały, a dokładniej wynosi $O((1-\alpha)^{-7/6})$, gdzie $\alpha=n/m$.

Zacznijmy od tego, że wynik $O((1-\alpha)^{-7/6})$ jest naprawdę dobry, gdyż $O((1-\alpha)^{-1})$ to oczekiwany czas jaki dostajemy, gdy zamiast szukać wolnego miejsca na $x$ używając kolejnych pozycji w tablicy $T$, korzystamy z w pełni losowej permutacji pozycji (patrz np. podręcznik Cormena i innych), co jest bardzo wyidealizowanym założeniem. Twierdzenie 2 jest dość zaskakujące, gdyż w świetle twierdzenia 1, które ,,z grubsza'' mówi, że 2-niezależność nie wystarcza, mogłoby się wydawać, że również $O(1)$-niezależność nie jest wystarczająca. Dodatkowo dziwi tu liczba 5, na pierwszy rzut oka wygląda to na niedokładne szacowanie, nieoptymalny wynik. Nic bardziej błędnego, gdyż w 2010r. Patrascu i Thorup wykazali, że

**Twierdzenie 3**  

Istnieje $4$-niezależna rodzina funkcji haszujących $\mathcal{H}$ taka, że jeśli w haszowaniu liniowym $h$ jest wybrana losowo z $\mathcal{H}$, to istnieje $S\subset U$ taki, że całkowity oczekiwany czas wstawienia wszystkich elementów z $S$  wynosi $\Omega(n\log n)$.

W dalszej części udowodnimy uproszczoną wersję twierdzenia 2.

**Twierdzenie 4**  

Jeśli w haszowaniu liniowym $h$ jest wybrana losowo z dowolnej 5-niezależnej rodziny funkcji haszujących $\mathcal{H}$, oraz $m\ge 3n$, to oczekiwany czas operacji słownikowych jest stały.

Zauważmy, że przyjęliśmy $\alpha \le \frac{1}{3}$. *Spójną składową* tablicy $T$ będziemy nazywać dowolny maksymalny ciąg kolejnych niepustych komórek tablicy $T$. Niech $cc(p)$ oznacza spójną składową zawierającą komórkę $p$, natomiast $\vert{}cc(p)\vert{}$ jej długość. Łatwo widać, że prawdziwy jest następujący lemat.

**Lemat 5**  

Czas działania każdej z operacji insert($x$),  lookup($x$) oraz delete($x$) można oszacować przez $O(\vert{}cc(h(x))\vert{})$. ♦

Weźmy dowolne $x\in U$. Zgodnie z lematem 5, aby wykazać twierdzenie 4 wystarczy, że oszacujemy przez pewną stałą oczekiwaną długość spójnej składowej zawierającej pozycję $h(x)$. Oczywiście zawsze istnieje takie $k$, że $\vert{}cc(h(x))\vert{} \in \{2^k, \ldots, 2^{k+1}-1\}$. Podzielmy komórki $T$ na równe bloki długości $2^{k-2}$.

Przypomnijmy, że notacja $\mathbf{1}_{\mathcal{E}}$, gdzie $\mathcal{E}$ jest dowolnym zdarzeniem oznacza indykator zdarzenia $\mathcal{E}$, czyli zmienną losową która przyjmuje wartość 1 gdy $\mathcal{E}$ zaszło i 0 w przeciwnym przypadku (formalnie, $\mathbf{1}_{\mathcal{E}}(\omega)=[\omega \in \mathcal{E}]$).

**Lemat 6**  

$\mathbb{E}[\vert{}h(S)\cap B\vert{}] = \alpha\vert{}B\vert{} = \frac{1}{3} \vert{}B\vert{}$

*Dowód*  

Korzystamy z liniowości wartości oczekiwanej i jednostajności $\mathcal{H}$, czyli faktu, że $\mathbb{P}[h(e)=b] = 1/m$ dla dowolnych $e\in U$, $b\in [m]$. Zauważmy, że $\mathbb{E}[\vert{}h(S)\cap B\vert{}] = \sum_{e \in S} \mathbb{E}[\mathbf{1}_{h(e)\in B}] = \sum_{e \in S} \mathbb{P}[h(e)\in B] = \sum_{e \in S}\sum_{b \in B}\mathbb{P}[h(e)=b] = n\cdot\vert{}B\vert{}\cdot \frac{1}{m}= \alpha\vert{}B\vert{} = \frac{1}{3} \vert{}B\vert{}$. ♦

Powiemy, że blok $B=\{i, i+1, \ldots, i + 2^{k-2} - 1\}$ jest *niebezpieczny*, gdy $\vert{}h(S) \cap B\vert{} \ge \frac{2}{3} \vert{}B\vert{}$. Uwaga!  Zauważmy, że nie liczymy tu ile komórek $B$ jest zajętych, a jedynie do ilu komórek $B$ haszują się elementy $S$.

**Lemat 7**  

 Jeśli $h(x)$ jest w spójnej składowej długości $\in \{2^k, \ldots, 2^{k+1}-1\}$ to jeden z $O(1)$ bloków przecinających $cc(h(x))$ jest niebezpieczny.

*Dowód*  

Oznaczmy kolejne bloki przecinające $cc(h(x))$ przez $B_1, \ldots, B_k$. Zauważmy, że $4 \le k \le 9$. Załóżmy przeciwnie, że wszystkie te bloki są bezpieczne. W szczególności oznacza to, że mniej niż $\frac{2}{3}2^{k-2}$ elementów haszujących do $B_1$ znajduje się w kolejnych blokach. W blokach $B_2$ i $B_3$ jest sumarycznie więcej niż $2\cdot \frac{1}{3}\vert{}B\vert{}$ komórek, które nie zawierają elementów haszujących do $B_2$ i $B_3$. To oznacza, że nie wszystkie z tych komórek zostaną zajęte przez elementy haszujące do $B_1$, a więc co najmniej jedna komórka pozostanie pusta (gdyż inne elementy nie mogą tam się pojawić). W takim razie $k\le 3$, sprzeczność. ♦

Załóżmy teraz, że znamy wartość $\rho=h(x)$ i chcemy oszacować prawdopodobieństwo, że $\vert{}cc(\rho)\vert{} \in \{2^k,\ldots,2^{k+1}-1\}$.  

Ponieważ $cc(\rho)$ przecina co najwyżej 9 bloków, to jest co najwyżej 17 bloków $B_1,\ldots, B_k$, które potencjalnie mogą przecinać się z $cc(\rho)$. Z lematu 7 wiemy, że jeśli $\vert{}cc(\rho)\vert{} \in \{2^k,\ldots,2^{k+1}-1\}$ to jeden z tych bloków jest niebezpieczny.  

Stąd, $$
\mathbb{P}[\vert{}cc(\rho)\vert{} \in \{2^k,\ldots,2^{k+1}-1\} \vert{} h(x)=\rho] \le \sum_{i=1}^k \mathbb{P}[\vert{}h(S)\cap B_i\vert{}\ge \frac{2}{3}\vert{}B_i\vert{} \vert{} h(x)=\rho].
$$  

Z symetrii i lematu 6 mamy dalej (*)

$$
\mathbb{P}[\vert{}cc(\rho)\vert{} \in \{2^k,\ldots,2^{k+1}-1\} \vert{} h(x)=\rho] =  O(1) \cdot \mathbb{P}[\vert{}h(S)\cap B\vert{}\ge \mathbb{E}(\vert{}h(S) \cap B\vert{})+\frac{1}{3}\vert{}B\vert{} \vert{} h(x)=\rho],
$$  

gdzie $B$ jest pewnym konkretnym blokiem długości $2^{k-2}$.

Od tej pory, dla uproszczenia zakładamy, że wszsytko jest warunkowane przez zdarzenie $h(x)=\rho$ i będziemy pomijać w prawdopodobieństwach (i wartościach oczekiwanych) napis ,,$\vert{} h(x)=\rho$''. Zauważmy, że przy tym warunkowaniu, rodzina $\mathcal{H}$ jest 4-niezależna.

Niech $X_e = \mathbf{1}_{h(e) \in B}$ oraz $X=\sum_{e\in S} X_e$. Przy takiej notacji, chodzi nam o to, żeby oszacować $\mathbb{P}[X > 2\mathbb{E}(X)]$. Narzuca się, żeby w celu oszacowania powyższego prawdopodobieństwa użyć nierówności Chernoffa, problem polega jednak na tym, że zmienne $X_e$ nie są niezależne (a jedynie 4-niezależne). Z nierówności Markowa dostajemy szacowanie przez $\frac{1}{2}$, lecz jak się później okaże jest ono o wiele za słabe. Kolejny pomysł to użycie nierówności Czebyszewa -- dałaby ona lepsze oszacowanie, lecz w dalszym ciągu niewystarczające. Okazuje się, że wystarczy zrobić jeden krok dalej -- mianowicie użyć następującego faktu:

**Nierówność czwartego momentu:** $\mathbb{P}[\vert{}X-\mathbb{E}X\vert{} > d] \le \frac{\mathbb{E}[(X-\mathbb{E}X)^4]}{d^4}.$

*Dowód nierówności*  

 Dowodzimy tak samo jak nierówność Czebyszewa, tylko podnosimy do 4-tej potęgi:  

$$
\mathbb{P}[\vert{}X-\mathbb{E}X\vert{} > d] = \mathbb{P}[(X-\mathbb{E}X)^4 > d^4] \le \frac{\mathbb{E}[(X-\mathbb{E}X)^4]}{d^4},
$$  

gdzie ostatnia nierówność wynika z nierówności Markowa.♦

Pozostaje jedynie oszacować czwarty moment, czyli $\mathbb{E}[(X-\mathbb{E}X)^4]$. Oznaczmy  

$Y_e = X_e - \mathbb{E}X_e$ oraz $Y=\sum_{e\in S} Y_e.$   

Wówczas $X-\mathbb{E}X = Y$ i interesuje nas $\mathbb{E}(Y^4)$. Mamy:  

$$
\mathbb{E}[Y^4] = \mathbb{E}\left[\left(\sum_{e\in S}Y_e\right)^4\right] = \sum_{e_1, e_2, e_3, e_4 \in S}\mathbb{E}(Y_{e_1}Y_{e_2}Y_{e_3}Y_{e_4}).
$$  

Zauważmy, że zmienne $Y_e$ są rownież 4-niezależne (gdyż $X_e$ są takie). Stąd, jeśli w ostatniej sumie $e_1, \ldots, e_4$ są parami różne, to zmienne $Y_{e_1}$, $Y_{e_2}$, $Y_{e_3}$, $Y_{e_4}$ są niezależne, czyli $\mathbb{E}(Y_{e_1}Y_{e_2}Y_{e_3}Y_{e_4}) = \mathbb{E}Y_{e_1}\mathbb{E}Y_{e_2}\mathbb{E}Y_{e_3}\mathbb{E}Y_{e_4}=0$, gdzie ostatnia równość bierze się stąd, że $\mathbb{E}Y_e = 0$. Ogólniej, jeśli $e_1 \not \in \{e_2, e_3, e_4\}$ to dwie zmienne  $Y_{e_1}$ i $Y_{e_2}Y_{e_3}Y_{e_4}$ są niezależne i dostajemy $\mathbb{E}(Y_{e_1}Y_{e_2}Y_{e_3}Y_{e_4}) = \mathbb{E}Y_{e_1}\mathbb{E}[Y_{e_2}Y_{e_3}Y_{e_4}]=0$. Stąd,  

$$
\mathbb{E}[Y^4] = \sum_{e\in S}\mathbb{E}[Y_e^4] + \sum_{e,f\in S; e < f}{4 \choose 2}\mathbb{E}[Y_e^2]\mathbb{E}[Y_f^2].
$$

Dla dowolnego $e$ i parzystego $j>0$ mamy  

$$
\begin{align}  

 \mathbb{E}[Y_e^j] &= \left(1-\tfrac{\vert{}B\vert{}}{m}\right)^j \tfrac{\vert{}B\vert{}}{m} + \left(-\tfrac{\vert{}B\vert{}}{m}\right)^j\left(1-\tfrac{\vert{}B\vert{}}{m}\right) =  

 \left(1-\tfrac{\vert{}B\vert{}}{m}\right)^j \tfrac{\vert{}B\vert{}}{m} + \left(\tfrac{\vert{}B\vert{}}{m}\right)^j\left(1-\tfrac{\vert{}B\vert{}}{m}\right) =\\  

&= \tfrac{\vert{}B\vert{}}{m} \left(1-\tfrac{\vert{}B\vert{}}{m}\right) \left(\left(1-\tfrac{\vert{}B\vert{}}{m}\right)^{j-1} + \left(\tfrac{\vert{}B\vert{}}{m}\right)^{j-1}\right) < \tfrac{\vert{}B\vert{}}{m}.  

\end{align}
$$  

Stąd, pamiętając o oznaczeniu $\alpha = n/m$,

$$
\mathbb{E}[Y^4] < n \frac{\vert{}B\vert{}}{m} + 6\frac{n^2}{2}\left(\frac{\vert{}B\vert{}}{m}\right)= \alpha\vert{}B\vert{} + 3(\alpha\vert{}B\vert{})^2 < 4(\alpha\vert{}B\vert{})^2.
$$

Stąd i z nierówności 4-tego momentu mamy, że (**)  

$$
\mathbb{P}[X>2\mathbb{E}X] = \mathbb{P}[X-\mathbb{E}X > \mathbb{E}X] < \frac{4(\alpha\vert{}B\vert{})^2}{(\alpha\vert{}B\vert{})^4} = \frac{4}{\alpha^4}\vert{}B\vert{}^{-2}=O(\vert{}B\vert{}^{-2}).
$$

Stąd i z (*) mamy

$$
\begin{align}  

\mathbb{E}[\vert{}cc(\rho)\vert{} \vert{} h(x)=\rho] &= \sum_l l\cdot \mathbb{P}[\vert{}cc(\rho)\vert{}=l \vert{} h(x)=\rho]\\  

& <  \sum_k 2^{k+1}\cdot \mathbb{P}[\vert{}cc(\rho)\vert{}\in\{2^k,\ldots,2^{k+1}-1\} \vert{} h(x)=\rho]\\  

& <^{(*), (**)}  \sum_k 2^{k+1}\cdot O(1)\cdot (2^{k-2})^{-2} \\  

& =  O(1) \sum_k 2^{-k} \\  

& =  O(1).  

\end{align}
$$

Ponieważ jest to prawdziwe dla dowolnego $\rho\in [m]$, więc $\mathbb{E}[\vert{}cc(h(x))\vert{}]=O(1)$, a tym samym dowód twierdzenia 4 jest zakończony.

**Literatura**

Wynik przedstawiony w tym wykładzie został po raz pierwszy opisany w pracy

A. Pagh, R. Pagh, M. Ruzic. [Linear probing with constant independence](http://arxiv.org/abs/cs/0612055). SIAM J. Comput., 39(3):1107–1120, 2009.

Użyty tutaj sposób prezentacji dowodów jest jednak w głównej mierze inspirowany [blogiem](http://infoweekly.blogspot.com/2010_01_01_archive.html) Mihai Pătraşcu oraz pracą

M. Pătraşcu, M. Thorup. [The Power of Simple Tabulation Hashing](http://arxiv.org/abs/1011.5200), Proc. 43rd ACM Symposium on Theory of Computing (STOC 2011).

Twierdzenie 3 zostało udowodnione w pracy

M. Patrascu and M. Thorup. On the -independence required by linear probing and minwise independence. Proc. ICALP 2010.
