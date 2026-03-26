---
title: "Ćwiczenia 2: Prawdopodobieństwo warunkowe"
source_url: "http://smurf.mimuw.edu.pl/node/82"
source_kind: "html"
---

<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>

# Ćwiczenia 2: Prawdopodobieństwo warunkowe

# Prawdopodobieństwo warunkowe, wzór iloczynowy

## Zadanie 1

Wybrano losowo jedną z trzech kartek, pomalowanych z obu stron: czarno-czarnej, czarno-białej i biało-białej, a następnie wybrano losowo jedną z jej stron. Jeśli ta strona jest czarna, to jakie jest prawdopodobieństwo tego, że druga też jest czarna?

## Zadanie 2

Wybieramy losowo numer z książki telefonicznej i pod niego dzwonimy. Osobę, która odbierze pytamy o to, czy ma dwójkę dzieci, a jeśli okaże się, że tak, to:

1. pytamy, czy ma syna. Jeśli okaże się, że tak, to jakie jest prawdopodobieństwo tego, że drugie dziecko też jest synem?
2. pytamy, czy któreś z dzieci ma na imię Jaś. Jeśli okaże się, że tak, to jakie jest prawdopodobieństwo tego, że drugie dziecko też jest synem?

**Uwaga:** W tym zadaniu trzeba oczywiście przyjąć pewne założenia co do częstości urodzin chłopców/dziewczynek oraz sposobu w jaki nadawane są im imiona.

## Zadanie 3

Na inżynierii oprogramowania 16 studentów ma być podzielonych na 4 zespoły. Wśród nich jest 4 pracujących, z doświadczeniem w projektowaniu dużych systemów. Jakie jest prawdopodobieństwo, że każdy z nich trafi do innej grupy?  

**Uwaga:** Nie rozwiązuj tego zadania wprost ze schematu klasycznego. Zamiast tego użyj pojęcia prawdopodobieństwa warunkowego.

## Zadanie 4

W 3-osobowym pojedynku trzech kawalerów A, B, C strzela do wybranego przez siebie celu w kolejności A, B, C cyklicznie, aż zostanie tylko jeden z nich. Przypuśćmy, że A trafia z prawdopodobieństwem 0.3, B trafia zawsze, a C z prawdopodobieństwem 0.5. Jaką strategię powinien przyjąć A?  

**Uwaga:** Zakładamy, że pozostali strzelcy postępują racjonalnie, tzn. tak aby maksymalizować swoje szanse przeżycia.

# Wzór na prawdopodobieństwo całkowite

## Zadanie 1

Grasz w turnieju szachowym, w którym z 50% graczy masz szansę wygrania 30%, z 25% graczy szansę 40% i z 25% graczy szansę 50%. Pierwszą partię rozgrywasz z losowym przeciwnikiem. Jakie sa twoje szanse wygrania?

## Zadanie 2 - model urnowy Polyi

W urnie są dwie kule: biała i czarna. $(n-2)$-krotnie losujemy kulę z urny, po czym wrzucamy ją z powrotem wraz z drugą kulą tego samego koloru. Jakie jest prawdopodobieństwo tego, że po zakończeniu losowań wśród $n$ kul dokładnie $k$ jest białych?

## Zadanie 3

Mamy $n$ urn. W $i$-tej urnie znajduje się $i-1$ kul białych i $n-i$ kul czarnych. Wybieramy losowo urnę, a następnie losujemy z niej dwie kule. Oblicz prawdopodobieństwo tego, że są to kule różnych kolorów jeśli losujemy:

1. bez zwracania,
2. ze zwracaniem.

# Twierdzenie Bayesa

## Zadanie 1 - problem Monty Halla

Gracz w teleturnieju wybiera jedną z 3 zasłon. Za jedną z zasłon jest ukryty samochód, za pozostałymi nic nie ma. Prowadzący odsłania jedną z pozostałych dwóch zasłon i pokazuje, że niczego za nią nie ma, po czym daje graczowi szansę na zmianę wyboru.

- Czy gracz powinien zmienić swój wybór?
- Jakie maksymalne prawdopodobieństwo sukcesu może uzyskać?
- Jak zmieni się sytuacja, jeśli zasłon jest 100, a prowadzący odsłania wszystkie poza wskazaną przez gracza i jedną inną?

**Uwaga:** Przeanalizowanie konkretnej sytuacji opisanej w zadaniu jest istotnie bardziej skomplikowane niż porównanie strategii "zawsze zmieniaj" ze strategią "nigdy nie zmieniaj". Czy widzisz dlaczego?

## Zadanie 2 - dylemat więźnia

Trzech więźniów A,B,C oczekuje na wykonanie kary śmierci.  Więzień A dowiedział się, że dwóch więźniów zostanie ułaskawionych, ale nie wie o których więźniów chodzi. Chciałby o to zapytać strażnika, ale ponieważ ten odmawia udzielania informacji dotyczących jego losu, prosi go o ujawnienie jednego z ułaskawionych więźniów spośród B,C. Strażnik ponownie odmawia twierdząc, że w ten sposób zmniejszyłby szansę A na ułaskawienie z 2/3 do 1/2, a nie chce tego robić, bo lubi A. Czy strażnik słusznie odmawia?

## Zadanie 3 - kontynuacja zadania 1 z poprzedniej serii

Przypuśćmy, że udało ci sie wygrać pierwsza partię. Jakie jest prawdopodobieństwo, że grałeś z graczem 3-go rodzaju?

## Zadanie 4

Mamy do dyspozycji 3 telefony. Wiemy, że jeden z nich zawsze działa, drugi nigdy nie działa (ale zjada monety), a trzeci działa z prawdopodobieństwem $p=\frac{1}{2}$ (w pozostałych przypadkach zjada monety). Próbujemy zadzwonić z jednego z automatów i zjada on monetę. Zmieniamy automat i udaje nam się zadzwonić. Próbujemy raz jeszcze (nie zmieniając telefonu) i znów się udaje. Jakie jest prawdopodobieństwo tego, że telefon z którego zadzwoniliśmy 2 razy jest tym, który zawsze działa? Czy odpowiedź zmieniłaby się, gdyby nie było w zadaniu pierwszej (nieudanej) próby?

## Zadanie 5

W sejmie mamy dwie partie. Posłowie partii A nigdy nie zmieniają zdania na żaden temat, a każdy z posłów partii B zmiena zdanie pomiędzy na temat głosowanej ustawy pomiędzy dwoma jej głosowaniami z prawdopodobieństwem $p$. Wiadomo, że posłowie partii A stanowią frakcję $f$ wszystkich posłów, pozostali pochodzą z partii B. Obserwujemy losowego posła i głosuje on w ten sam sposób w dwóch kolejnych głosowaniach. Jakie jest prawdopodobieństwo tego, że pochodzi on z partii A?

## Zadanie 6

Dane są dwie urny. W pierwszej urnie znajdują się 2 czerwone kule i 1 czarna, w drugiej 101 czerwonych i 100 czarnych. Ktoś wybiera losowo urnę (nie wiemy którą), z wybranej urny losuje kulę i mówi nam jakiego jest ona koloru. Następnie z tej samej urny losowana jest druga kula, przy czym możemy zażądać, aby uprzednio pierwsza kula wróciła do urny. Naszym zadaniem jest zgadnięcie z której urny są losowane kule. Jak wygląda optymalna strategia?  

**Uwaga:** To zadanie jest dość pracochłonne.

## Zadanie 7 - doświadczenie Laplace'a

Danych jest $N+1$ urn.  W $i$-tej urnie znajduje się $i$ kul białych i $N-i$ kul czerwonych, dla $i=0,...,N$. Wybieramy losowo urnę, a następnie $n$-krotnie losujemy z wybranej urny jedną kulę ze zwracaniem. Przypuśćmy, że za każdym razem była to kula czerwona. Jakie jest prawdopodobieństwo tego, że kolejna kula wylosowana z tej urny też będzie czerwona?  

**Uwaga:** Laplace użył tego rozumowania do argumentowania na temat prawdopodobieństwa tego, że słońce następnego dnia wzejdzie na podstawie tego, że wzeszło odpowiednio wiele razy wcześniej. Co sądzisz o tym "zastosowaniu"?
