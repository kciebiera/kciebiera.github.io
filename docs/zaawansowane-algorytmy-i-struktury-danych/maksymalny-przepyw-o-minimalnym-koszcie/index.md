---
layout: 'default'
title: 'Przepływ maksymalny o minimalnym koszcie'
page_url: 'http://smurf.mimuw.edu.pl/node/1119'
source_url: 'http://smurf.mimuw.edu.pl/node/1119'
source_kind: 'html'
uses_math: true
render_with_liquid: false
rendered_file: 'lectures/zaawansowane-algorytmy-i-struktury-danych/maksymalny-przepyw-o-minimalnym-koszcie/rendered.html'
---

# Przepływ maksymalny o minimalnym koszcie

## Definicja problemu

W tym wykładzie wymagamy od Czytelnika znajomości wykładu o maksymalnym przepływie.

Egzemplarz problemu maksymalnego przepływu o minimalnym koszcie stanowi graf skierowany $G=(V,E)$, funkcje $c:E\rightarrow\mathbb{R}_{\ge 0}$ i $a:E\rightarrow \mathbb{R}\}$ oraz dwa wyróżnione wierzchołki $s$ i $t$. Podobnie jak w wykładzie o maksymalnym przepływie, $c$ nazywamy funkcją przepustowości. Natomiast $a$ jest funkcją kosztu oraz $a(e)$ nazywamy kosztem krawędzi $e$. W grafie $G$ dopuszczamy możliwość istnienia wielu krawędzi o tym samym początku i końcu. Jest to naturalne założenie, gdyż koszty tych krawędzi mogą się różnić.

*Przepływem* nazywamy dowolną funkcję $f:E\rightarrow\mathbb{R}_{\ge 0}$ taką, że:

- **(warunek przepustowości)** $0 \le f(e) \le c(e)$ dla każdej krawędzi $e \in E$.
- **(warunek zachowania przepływu)** $\sum_{(u,v)\in E} f(u,v) = \sum_{(v,u)\in E}f(v,u)$ dla każdego wierzchołka $v \in V\setminus\{s,t\}$.

Zauważmy, że powyższa definicja różni się nieco od definicji przepływu z wykładu o maksymalnym przepływie. Różnica polega na tym, że w tym rozdziale przepływ nie spełnia warunku skośnej symetryczności. Konieczność użycia nowej definicji wynika z faktu istnienia wielu krawędzi o tym samym początku i końcu. Oczywiście obie definicje są równoważne dla grafów, w których dla dowolnych wierzchołków $u$ i $v$ jest co najwyżej jedna krawędź spośród krawędzi $(u,v)$, $(v,u)$. Używana w tym rozdziale definicja jest bardziej naturalna i wygodniejsza podczas implementacji algorytmów, choć mniej wygodna przy dowodzeniu.

*Kosztem przepływu $f$* nazywamy liczbę $cost(f) = \sum_{e\in E} a(e) f(e)$. Zgodnie z tą definicją, wartość $a(e)$ interpretujemy jako koszt przesłania 1 jednostki przepływu po krawędzi $e$. W tym rozdziale zajmujemy się następującym problemem optymalizacyjnym.

**Problem 1**  

Znaleźć w sieci $(G,a,c,s,t)$ przepływ maksymalny o minimalnym koszcie (wśród przepływów maksymalnych).

Oprócz problemu 1, nieco inne zagadnienie wydaje się równie naturalne:

**Problem 2**  

Dla danej sieci $(G,a,c,s,t)$ i liczby $w \in \mathbb{R_{\ge 0}}$ znaleźć przepływ o minimalnym koszcie wśród przepływów o wartości $w$.

Nietrudno dostrzec, że problemy 1 i 2 są równoważne. Oczywiście skoro umiemy wyznaczyć wartość maksymalnego przepływu, to problem 1 sprowadza się do problemu 2. Z drugiej strony, jeśli umiemy rozwiązać problem 1, mając dany egzemplarz $(G,a,c,s,t,w)$ problemu 2 wystarczy rozwiązać problem 1 dla egzemplarza $(G',a,c,s',t)$, gdzie graf $G'$ powstaje z $G$ przez dodanie nowego wierzchołka $s'$ oraz krawędzi $(s',s)$ o koszcie 0 i przepustowości $w$. W dalszej części wykładu będziemy mówić już wyłącznie o problemie 1.

## Sieć residualna

Wprowadzenie kosztów krawędzi do naszego problemu wymusza nieco inną definicję sieci residualnej, jednak jej rola pozostaje ta sama. Dla danego przepływu $f$ w sieci $(G=(V,E),a,c,s,t)$, sieć residualna $G_f$ ma ten sam zbiór wierzchołków $V$ co sieć $G$. Krawędzie $G_f$ chcemy określić w taki sposób, że przesłanie jednostki przepływu w $G_f$ po krawędzi $e=(u,v)$ odpowiada powiększeniu $f$ wzdłuż krawędzi $e$ o tę jednostkę lub pomniejszeniu $f$ o tę jednostkę wzdłuż odwrotnej krawędzi, czyli wzdłuż $(v,u)$.

Zgodnie z powyższą intuicją, dla każdej krawędzi $e\in E$ takiej, że $f(e) < c(e)$, w zbiorze $E_f$ sieci $G_f$ umieszczamy krawędź $e$ o przepustowości residualnej $c_f(e) = c(e)-f(e)$ i koszcie $a_f(e)=a(e)$. Natomiast dla każdej krawędzi $e=(u,v)\in E$ takiej, że $f(e) > 0$, w $E_f$ umieszczamy krawędź $\text{rev}(e)=(v,u)$ o przepustowości residualnej $c_f(\text{rev}(e)) = f(e)$ i koszcie $a_f(\text{rev}(e))=-a(e)$.

Jeśli $f$ jest przepływem w $G$, natomiast $g$ jest przepływem w $G_f$, to $f+g$ definiujemy jako przepływ w $G$, gdzie dla $e\in E$ określamy $(f+g)(e) = f(e) + g(e) - g(\text{rev}(e))$. Łatwo widać, że $\vert{}f+g\vert{} = \vert{}f\vert{}+\vert{}g\vert{}$ oraz $cost(f+g)=cost(f)+cost(g)$.

Podobnie, jeśli $f$ i $f'$ są przepływami w $G$ to $f'-f$ definiujemy jako następującą funkcję $(f'-f):E_f \rightarrow\mathbb{R}_{\ge 0}$. Dla $e \in E$, jeśli $f'(e) > f(e)$, to $(f'-f)(e)=f'(e)-f(e)$, natomiast jeśli $f'(e) < f(e)$, to $(f'-f)(\text{rev}(e))=f(e)-f'(e)$. Na pozostałych elementach $E_f$ funkcja $f'-f$ ma wartość 0. Łatwy dowód poniższego lematu pozostawiamy Czytelnikowi.

**Lemat 3**  

Funkcja $(f'-f)$ jest przepływem w sieci residualnej $G_f$ o wartości $\vert{}f'-f\vert{}=\vert{}f'\vert{}-\vert{}f\vert{}$ i koszcie $cost(f'-f)=cost(f')-cost(f)$. ♦

## Algorytm usuwania ujemnych cykli

Następujący lemat jest często wykorzystywany w problemach związanych z przepływami.

**Lemat 4**  

Każdy przepływ można rozłożyć na sumę cykli i ścieżek prostych od $s$ do $t$.

*Dowód*  

Niech $f$ będzie dowolnym przepływem. Użyjemy indukcji po liczbie krawędzi $e$ takich, że $f(e) > 0$.

Baza indukcji. Jeśli nie ma takich krawędzi to teza lematu jest prawdziwa (suma pusta).

Krok indukcyjny. Jeśli istnieje cykl $C$, którego krawędzie mają dodatni przepływ, to definiujemy przepływ $g$, który jest zerowy wszędzie poza $C$, a na wszystkich krawędziach $C$ ma wartość $min_{e\in C} f(e)$. Żądaną w tezie rodzinę cykli i ścieżek otrzymujemy rozkładając na podstawie założenia indukcyjnego przepływ $f-g$ na cykle i ścieżki i dodając do tej rodziny cykl $C$.

Załóżmy więc, że nie istnieje opisany wyżej cykl. Niech $e_0=(v_0,v_1)$ będzie dowolną krawędzią taką, że $f(e_0)>0$. Wówczas z warunku zachowania przepływu albo $v_1=t$ albo istnieje krawędź $e_1=(v_1, v_2)$ taka, że $f(e_1)>0$. Ponownie, albo $v_2=t$ albo istnieje krawędź $e_2=(v_2,v_3)$... itd. Ponieważ wykluczyliśmy cykle, ta marszruta będzie ścieżką prostą, a więc ponieważ graf jest skończony, musi zakończyć się w $t$. Podobnie, z warunku zachowania przepływu albo $v_0=s$ albo istnieje krawędź $e_{-1}=(v_{-1}, v_0)$ taka, że $f(e_{-1})>0$. Ponownie, albo $v_{-1}=s$ albo istnieje krawędź $e_{-2}=(v_{-2},v_{-1})$... itd: otrzymujemy ścieżkę prostą od $s$ do $v_0$. Suma tych dwóch ścieżek musi być ścieżką prostą od $s$ do $t$. Korzystając z założenia indukcyjnego analogicznie jak w przypadku cyklu, dostajemy tezę lematu. ♦

Jeśli $X$ jest zbiorem krawędzi, to kosztem $X$ nazwiemy $a(X)=\sum_{e\in X}a(e)$. Przez koszt ścieżki lub cyklu rozumiemy koszt odpowiedniego zbioru krawędzi.

**Lemat 5**  

Niech $f$ będzie przepływem w $(G,a,c,s,t)$. Wówczas $f$ jest przepływem o minimalnym koszcie wśród przepływów o wartości $\vert{}f\vert{}$ wtw. gdy $G_f$ nie zawiera cykli o ujemnym koszcie.

*Dowód*  

Implikacja $(\rightarrow)$ jest oczywista, gdyż dodanie do $f$ cyklu o ujemnym koszcie nie zmienia wartości $f$, natomiast zmniejsza jego koszt.

Zajmijmy się implikacją $(\leftarrow)$. Niech $f'$ będzie dowolnym przepływem o wartości $\vert{}f'\vert{}=\vert{}f\vert{}$. Z lematu 3 $f'-f$ jest przepływem w $G_f$ o wartości 0. Z lematu 4 $f'-f$ rozkłada się na sumę cykli i ścieżek od $s$ do $t$. Ponieważ $\vert{}f'-f\vert{}=0$, więc w tym rozkładzie nie ma żadnych ścieżek. Z założenia, każdy z pozostałych cykli ma nieujemny koszt. Stąd, $cost(f'-f) \ge 0$, a więc z lematu 3, $cost(f')\ge cost(f)$. ♦

Korzystając z powyższego lematu dostajemy prosty algorytm znajdowania maksymalnego przepływu o minimalnym koszcie:

1. Znajdź maksymalny przepływ $f$ w $(G,c,s,t)$, np. algorytmem Forda-Fulkersona (patrz odpowiedni [wykład](http://wazniak.mimuw.edu.pl/index.php?title=Zaawansowane_algorytmy_i_struktury_danych%2FWyk%C5%82ad_9)),
2. Tak długo, jak $G_f$ zawiera cykl $C$ o ujemnym koszcie (taki cykl wykrywamy za pomocą algorytmu Bellmana-Forda -- patrz odpowiedni [wykład](http://wazniak.mimuw.edu.pl/index.php?title=Zaawansowane_algorytmy_i_struktury_danych%2FWyk%C5%82ad_5)), znajdź taki cykl, i zbuduj przepływ $g$ w $G_f$, który dla każdej krawędzi $e \in E(C)$ ma $g(e)=\min_{e\in E(C)}c_f(e)$, a poza $E(C)$ jest zerowy, a następnie wykonaj $f := f + g$.

Jako ćwiczenie pozostawiamy pokazanie, że algorytm Bellmana-Forda można rozszerzyć tak, aby nie tylko wykrywać istnienie ujemnych cykli, ale także znaleźć pewien taki cykl w czasie $O(\vert{}V\vert{}\cdot \vert{}E\vert{})$.

Podobnie jak algorytm Forda-Fulkersona, powyższy algorytm nie ma własności stopu gdy przepustowości są liczbami rzeczywistymi. Jeśli jednak przepustowości i koszty są wymierne, algorytm zatrzyma się, zwracając poprawną odpowiedź. Czas jego działania będzie pseudowielomianowy (tzn. zależy wielomianowo od $\vert{}V\vert{}$ oraz $\max_{e\in E} c(e)$ i $\max_{e\in E} a(e)$, zakładając że $c$ i $a$ są całkowitoliczbowe). W kolejnym rozdziale przedstawimy bardziej efektywny algorytm, działający w szczególnym przypadku, gdy dana na wejściu sieć nie ma cykli o ujemnym koszcie.

## Algorytm najtańszej ścieżki

Załóżmy, że w sieci $(G,a,c,s,t)$ danej na wejściu nie ma cykli o ujemnym koszcie. Wówczas możemy zastosować następujący naturalny algorytm.

1. Niech $f$ będzie przepływem zerowym.
2. Tak długo, jak $G_f$ zawiera ścieżkę powiększającą, powiększ $f$ wzdłuż **najtańszej** ścieżki powiększającej $P$: przesyłamy $\min_{e\in E(P)} c_f(e)$ jednostek przepływu.

### Poprawność algorytmu

Zacznijmy od udowodnienia poprawności powyższego algorytmu, zwanego algorytmem najtańszej ścieżki. Z twierdzenia o maksymalnym przepływie i minimalnym przekroju, otrzymujemy że uzyskany po zakończeniu algorytmu przepływ $f$ faktycznie jest maksymalny (zauważmy, ze nasz algorytm jest po prostu pewną implementacją algorytmu Forda-Fulkersona). Pozostaje pokazać, że $f$ ma minimalny koszt. Wynika to z następującego niezmiennika pętli w naszym algorytmie:

**Niezmiennik 1:** $f$ ma minimalny koszt wśród przepływów o wartości $\vert{}f\vert{}$.

Zauważmy, że niezmiennik 1 jest spełniony na początku algorytmu, dla przepływu zerowego, bowiem dowolny przepływ o wartości 0 z lematu 4 rozkłada się na sumę cykli, a wiemy że cykle te mają nieujemne koszty: stąd, dowolny przepływ zerowy ma nieujemny koszt. Udowodnimy teraz, że niezmiennik 1 pozostaje zachowany po obrocie pętli.

**Lemat 6**  

Niech $f$ będzie pewnym przepływem o minimalnym koszcie spośród przepływów o wartości $\vert{}f\vert{}$.  Niech $g$ będzie przepływem w sieci residualnej $G_f$, który jest niezerowy tylko na krawędziach pewnej najtańszej ścieżki $P$ od $s$ do $t$. Wówczas $f+g$ jest najtańszym przepływem o wartości $\vert{}f+g\vert{}$.

*Dowód*  

Niech $f'$ będzie dowolnym przepływem o wartości $\vert{}f+g\vert{}$. Pokażemy, że $cost(f')\ge cost(f+g)$.

Z lematu 3, $f'-f$ jest przepływem w $G_f$, o wartości $\vert{}f'-f\vert{}=\vert{}f'\vert{}-\vert{}f\vert{}=\vert{}f+g\vert{}-\vert{}f\vert{}=\vert{}g\vert{}$. Z lematu 4, $f'-f$ można rozbić na sumę cykli $C_1,\ldots,C_k$ i ścieżek $P_1\ldots,P_m$ od $s$ do $t$. Ponieważ $f$ ma minimalny koszt, więc na mocy lematu 5 w $G_f$ nie ma cykli o ujemnym koszcie, więc każdy z cykli $C_1,\ldots,C_k$ ma nieujemny koszt. Z kolei dla każdego $i=1,\ldots,m$ mamy $a(P_i) \ge a(P)$, gdyż $P$ jest najtańszą ścieżką. Oznaczmy przez $\vert{}C_i\vert{}$ wartość przepływu, który płynie po cyklu $C_i$, analogicznie definiujemy $\vert{}P_i\vert{}$. Razem dostajemy

$$
cost(f'-f) = \sum_{i=1}^k \vert{}C_i\vert{} a(C_i) + \sum_{i=1}^m \vert{}P_i\vert{} a(P_i) \ge \sum_{i=1}^m \vert{}P_i\vert{} a(P_i) \ge a(P) \sum_{i=1}^m \vert{}P_i\vert{} \ge a(P) \vert{}f'-f\vert{} = a(P) \vert{}g\vert{} = cost(g).
$$

Ponieważ z lematu 3, $cost(f'-f) = cost(f') - cost(f)$, otrzymujemy $cost(f') \ge cost(f)+cost(g) = cost(f+g)$. ♦

### Złożoność algorytmu

Złożoność algorytmu zależy od tego, w jaki sposób wyszukujemy najtańszą ścieżkę w grafie. Zauważmy, że w sieci residualnej pojawiają się krawędzie o ujemnym koszcie, nie możemy więc użyć algorytmu Dijkstry. Jeśli skorzystamy z algorytmu Bellmana-Forda otrzymamy złożoność $O(\vert{}f\vert{}\cdot\vert{}V\vert{}\cdot\vert{}E\vert{})$.

Aby poprawić złożoność algorytmu skorzystamy z tricku podobnego jak w algorytmie Johnsona ((patrz odpowiedni [wykład](http://wazniak.mimuw.edu.pl/index.php?title=Zaawansowane_algorytmy_i_struktury_danych%2FWyk%C5%82ad_6))).  Przypomnijmy, że jeśli mamy funkcję $\pi:V\rightarrow\mathbb{R}$ oraz dla każdej krawędzi $e=(u,v)$ określimy $a_{\pi}(e) = a(e) + \pi(u) - \pi(v)$ to wówczas najtańsza ścieżka względem wagi $a_{\pi}$ jest także najtańszą ścieżką względem wagi $a$ (lemat 8 z powyższego wykładu). Co więcej, jeśli $\pi$ jest potencjałem, to  $a_{\pi}(e)\ge 0$ dla każdej krawędzi $e$. Pomysł polega na tym, żeby najtańsze ścieżki obliczać algorytmem Dijkstry w sieci o wagach $a_{\pi}$. Początkowy potencjał $\pi$ wyznaczamy za pomocą algorytmu Bellmana-Forda, podobnie jak to było w algorytmie Johnsona. Okazuje się, że po zakończonej iteracji, potencjał dla nowej sieci residualnej można łatwo obliczyć w czasie liniowym.

Zmodyfikowany algorytm wygląda następująco.

1. Niech $f$ będzie przepływem zerowym.
2. Za pomocą algorytmu Bellmana-Forda, dla każdego wierzchołka oblicz $\pi(v)$, koszt najtańszej ścieżki od $s$ do $v$.
3. Tak długo, jak $G_f$ zawiera ścieżkę powiększającą, wykonuj:
  - Uruchom algorytm Dijkstry z wierzchołka $s$ w grafie $G_f$ z kosztami krawędzi $a_{\pi}$. Algorytm ten znajdzie:
    - najtańszą ścieżkę powiększającą $P$ względem kosztów $a_{\pi}$, oraz
    - dla każdego wierzchołka $v \in V$ obliczy $\delta_{\pi}(v)$, koszt najtańszej ścieżki od $s$ do $v$.

  - Powiększ $f$ wzdłuż $P$ przesyłając $\min_{e\in E(P)} c_f(e)$ jednostek przepływu.
  - Dla każdego wierzchołka $v \in V$ przypisz $\pi(v) := \pi(v) + \delta_{\pi}(v)$.

Poprawność algorytmu wynika z następującego niezmiennika.

**Niezmiennik 2:** $\pi$ jest potencjałem w grafie $G_f$.

Na początku, gdy $\pi$ jest po prostu funkcją odległości $\delta$ w grafie, w którym za długość krawędzi przyjmujemy jej koszt, niezmiennik jest oczywisty, gdyż dla każdej krawędzi $u,v$, mamy $\delta(v) \le \delta(u) + a(u,v)$.

*Dowód, że niezmiennik jest zachowany po obrocie pętli*  

Niech $f$ będzie przepływem przed iteracją, $f'$ przepływem po iteracji. Zakładamy, że $\pi$ jest potencjałem w $G_f$ tzn. dla każdej krawędzi $e=(u,v)\in E_f$ mamy $a_{\pi}(e)\ge 0$. Pokażemy, że $\pi'=\pi+\delta$ jest potencjałem w $G_{f\,^\prime}$, tzn. że dla każdej krawędzi $e=(u,v)\in E_{f\,'}$ mamy $a_{\pi'}(e)\ge 0$. Rozważmy dowolną krawędź $e=(u,v)\in E_{f\,'}$.

Jeśli $e \in E_f$, to $\delta_{\pi}(v) \le \delta_{\pi}(u) + a_{\pi}(e)$. Stąd, $0 \le a_{\pi}(e) + \delta_{\pi}(u) - \delta_{\pi}(v) = a(e) + \pi(u) - \pi(v) + \delta_{\pi}(u) - \delta_{\pi}(v) = a(e) + \pi'(u) - \pi'(v) = a_{\pi'}(e)$.

Jeśli natomiast $e \not \in E_f$, to wiemy, że podczas iteracji przesyłano przepływ po krawędzi $\text{rev}(e)\in E_f$. Stąd, krawędź $\text{rev}(e)=(v,u)$ leży na najtańszej ścieżce od $s$ do $t$ w $G_f$ z kosztami $a_{\pi}$, a więc $\delta_{\pi}(u)=\delta_{\pi}(v)+a_{\pi}(\text{rev}(e)) = \delta_{\pi}(v) + a(\text{rev}(e)) + \pi(v) - \pi(u)$. Stąd, $\pi'(v) - \pi'(u) + a(\text{rev}(e)) = 0$. Ponieważ  $a(\text{rev}(e))=- a(e)$ otrzymujemy $\pi'(u) - \pi'(v) + a(e) = 0$, czyli $a_{\pi'}(e)=0$. ♦

Jeśli algorytm Dijkstry zaimplementujemy z użyciem kopców Fibonacciego to jedna iteracja naszego algorytmu zajmie czas $O(\vert{}V\vert{}\log\vert{}V\vert{}+\vert{}E\vert{})$. Otrzymujemy zatem następujące twierdzenie.

**Twierdzenie**  

W sieci bez cykli o ujemnym koszcie można znaleźć najtańszy maksymalny przepływ $f$ w czasie $O(\vert{}V\vert{}\cdot\vert{}E\vert{}+\vert{}f\vert{}(\vert{}V\vert{}\log\vert{}V\vert{}+\vert{}E\vert{}))$.

## Najcięższe / najlżejsze skojarzenia w grafach dwudzielnych

Niech $G=(V,E)$ będzie grafem skierowanym oraz niech $w:E \rightarrow \mathbb{R}$ będzie dowolną funkcją wag krawędzi. Wagą skojarzenia $M$ nazywamy liczbę $w(M) = \sum_{e\in M} w(e)$. W problemie najlżejszego (najcięższego) skojarzenia należy wyznaczyć skojarzenie doskonałe (tzn. takie, że każdy wierzchołek jest skojarzony) o minimalnej (maksymalnej) wadze. Oczywiście problem najcięższego skojarzenia redukuje się do problemu najlżejszego skojarzenia: wystarczy przemnożyć wagi krawędzi przez $-1$.

Łatwo zauważyć, że problem najlżejszego skojarzenia **w grafie dwudzielnym** $G=(U,V,E)$ redukuje się do problemu maksymalnego przepływu o minimalnym koszcie. Mianowicie, budujemy sieć o zbiorze wierzchołków $U \cup V \cup \{s,t\}$. Dla każdego wierzchołka $u\in U$, umieszczamy w sieci krawędź $(s,u)$ o przepustowości 1 i koszcie 0. Podobnie, dla każdego wierzchołka $v\in V$, umieszczamy w sieci krawędź $(v,t)$ o przepustowości 1 i koszcie 0. W końcu, dla każdej krawędzi $uv \in E$ takiej że $u\in U$ oraz $v\in V$, umieszczamy w sieci krawędź $(u, v)$ o przepustowości 1 i koszcie $w(u,v)$. Łatwo sprawdzić, że w takiej sieci nie ma cykli o ujemnym koszcie oraz że dowolny przepływ $f$ o wartości $\vert{}f\vert{}=\vert{}U\vert{}=\vert{}V\vert{}$ odpowiada skojarzeniu (jakiemu?) o wadze $cost(f)$.

**Wniosek**  

Najlżejsze (najcięższe) skojarzenie doskonałe w grafie dwudzielnym o $n$ wierzchołkach i $m$ krawędziach można wyznaczyć w czasie $O(nm+n^2\log n)$.

Okazuje się, że nawet w dowolnym grafie możliwe jest wyznaczenie najlżejszego skojarzenia doskonałego w czasie wielomianowym. Algorytm ten nie korzysta ze sprowadzenia do przepływów i wykracza poza zakres tego wykładu.
