---
title: "Ćwiczenia 1: Schemat klasyczny"
source_url: "http://smurf.mimuw.edu.pl/node/81"
source_kind: "html"
---

<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@4/tex-mml-chtml.js"></script>

# Ćwiczenia 1: Schemat klasyczny

# Zadania elementarne

## Zadanie 1

Losujemy 2 kule spośród $c$ czerwonych i $b$ białych. Jakie jest prawdopodobieństwo wylosowania kul różnych kolorów?

## Zadanie 2

90-osobowy rocznik został podzielony losowo na 3 równoliczne potoki. Jakie jest prawdopodobieństwo, że Jaś i Małgosia znajdą się w tym samym potoku?

## Zadanie 3

Jakie jest prawdopodobieństwo, że w losowym ustawieniu $n$ wież na szachownicy $n \times n$ żadne 2 wieże się nie atakują?

## Zadanie 4

Rozdajemy 7 kart ze standardowej talii 52 kart. Jakie jest prawdopodobieństwo tego, że wśród tych kart są:

1. dokładnie 3 asy?
2. dokładnie 2 króle?
3. dokładnie 3 asy lub dokładnie 2 króle?

## Zadanie 5

Rzucamy trzema kostkami. Sumy 11 i 12 można uzyskać na tyle samo sposobów. Czy są one równie prawdopodobne?

## Zadanie 6

Jaś i Małgosia rzucają monetami: Jaś rzuca $n$ razy, a Małgosia $n+1$ razy. Jakie jest prawdopodobieństwo tego, że Małgosi wypadnie więcej orłów niż Jasiowi?

# Zadania trudniejsze i bardziej koncepcyjne

## Zadanie 1

W klasie jest $n$ osób. Obliczyć prawdopodobieństwo tego, że pewne dwie osoby mają urodziny tego samego dnia? Jak duże musi być $n$, aby to prawdopodobieństwo było większe niż $\frac{1}{2}$?

Wskazówka  

 $e^x \approx 1 + x$ dla małych $x$.

## Zadanie 2

10 osób wsiadło na parterze 10-piętrowego budynku do pustej windy. Jakie jest prawdopodobieństwo tego, że każda wysiądzie na innym piętrze? Zakładamy, że każdy układ wysiadających jest równie prawdopodobny)

## Zadanie 3

$n$ listów włożono losowo do $n$ zaadresowanych kopert (każdy list ma swojego docelowego adresata; wszyscy adresaci są różni). Jakie jest prawdopodobieństwo tego, że:

1. żaden list nie trafił do właściwej koperty?
2. dokładnie $k$ listów trafiło do właściwych kopert?

Wskazówka  

 Zasada włączeń i wyłączeń.

## Zadanie 4 - optymalizacja wyboru żony (ew. męża)

Przyjmijmy, że mamy szansę spotkać w życiu $n$ kobiet - potencjalnych kandydatek na żonę (ew. mężczyzn - kandydatów na męża). Każdą z kandydatek możemy porównać do każdej z wcześniejszych kandydatek (biorąc pod uwagę wiek, charakter, poczucie humoru, urodę etc). Jesli nie wybierzemy danej kandydatki to już więcej jej nie zobaczymy. Naszym celem jest opracowanie strategii, która maksymalizuje prawdopodobieństwo wyboru najlepszej kandydatki. Ograniczymy się przy tym do strategii następującej postaci:

1. odrzuć $k-1$ pierwszych kandydatek, a następnie
2. wybierz pierwszą kandydatkę lepszą od wszystkich wcześniejszych.

W tym zadaniu nie musisz dowodzić, że optymalna strategia ma taką właśnie postać, choć z pewnością warto się nad tym zastanowić.  

Twoim zadaniem jest ustalenie:

1. Jakie prawdopodobieństwo sukcesu gwarantuje ta strategia?
2. Jaka jest optymalna wartość parametru $k$ i jakie prawdopodobieństwo sukcesu odpowiada tej wartości?

Wskazówka  

 Interesują nas takie permutacje $p_1,\ldots,p_n$ dla których, dla pewnego $s, k\le s \le n$:

- $n=\max_i p_i = p_s$ ,
- $\max_{i < s} p_i= \max_{i < k} p_i$.

## Zadanie 5

Co jest bardziej prawdopodobne:

- uzyskanie co najmniej 1 szóstki w 6 rzutach?
- uzyskanie co najmniej 2 szóstek w 12 rzutach?
- uzyskanie co najmniej 3 szóstek w 18 rzutach?
