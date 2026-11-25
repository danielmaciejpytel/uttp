<p align="center">
  <img src="docs/media/logo.png" alt="Ultimate Third Person Project" width="820">
</p>

<p align="center"><b>Płynność. Responsywność. Immersja.</b><br>
Lokomocja oparta na Motion Matching, naturalne obroty ciała oraz precyzyjne celowanie i walka w trzeciej osobie w Cyberpunk 2077.</p>

<p align="center">
  <img src="https://img.shields.io/badge/Cyberpunk_2077-2.5-FCEE0A?style=for-the-badge&labelColor=1a1a1a" alt="Cyberpunk 2077 2.5">
  <img src="https://img.shields.io/badge/Phantom_Liberty-required-FF2A6D?style=for-the-badge&labelColor=1a1a1a" alt="Wymagane Phantom Liberty">
  <img src="https://img.shields.io/badge/version-0.2.0-00E5FF?style=for-the-badge&labelColor=1a1a1a" alt="Wersja 0.2.0">
</p>
<p align="center">
  <img src="https://img.shields.io/badge/RED4ext-1.31.1-FCEE0A?style=flat-square&labelColor=1a1a1a" alt="RED4ext 1.31.1">
  <img src="https://img.shields.io/badge/redscript-0.5.32-00E5FF?style=flat-square&labelColor=1a1a1a" alt="redscript 0.5.32">
  <img src="https://img.shields.io/badge/CET-1.38.1-FF2A6D?style=flat-square&labelColor=1a1a1a" alt="CET 1.38.1">
  <img src="https://img.shields.io/badge/Codeware-1.21.4-FCEE0A?style=flat-square&labelColor=1a1a1a" alt="Codeware 1.21.4">
  <img src="https://img.shields.io/badge/ArchiveXL-1.28.8-00E5FF?style=flat-square&labelColor=1a1a1a" alt="ArchiveXL 1.28.8">
  <img src="https://img.shields.io/badge/Input_Loader-required-FF2A6D?style=flat-square&labelColor=1a1a1a" alt="Input Loader">
</p>

<p align="center">
  <b>Polski</b> · <a href="README.md">English</a> ·
  <a href="#szybki-start">Szybki start</a> ·
  <a href="#co-nowego-w-020">Co nowego</a> ·
  <a href="#zobacz-różnicę">Zobacz różnicę</a> ·
  <a href="#funkcje">Funkcje</a> ·
  <a href="#wymagania">Wymagania</a> ·
  <a href="#jak-zgłosić-problem">Zgłoś problem</a> ·
  <a href="CHANGELOG.md">Lista zmian</a> ·
  <a href="#dla-programistów">Dla programistów</a>
</p>

<p align="center"><img src="docs/media/divider.svg" width="100%"></p>

## Szybki start

> [!NOTE]
> Wersja 0.2.0 przerabia poruszanie się pieszo: sposób doboru i tempo animacji, ruszanie, zatrzymania, obroty i uniki, stawianie stóp na nierównym terenie, znacznie lżejsze pochylenie oraz zachowanie przy celowaniu i obrotach. Przy aktualizacji ustawienia moda są jednorazowo przywracane do nowych wartości domyślnych. To nadal wczesna wersja; zobacz [listę zmian](CHANGELOG.md). Prace nad 0.2.5, skupione na lokomocji i pokonywaniu przeszkód, już trwają. Uwagi są bardzo mile widziane.

1. Zainstaluj [wymagane komponenty](#wymagania): RED4ext, redscript, Cyber Engine Tweaks, Codeware, ArchiveXL i Input Loader.
2. Zainstaluj UTPP przez Vortex (**Mod manager download**) albo rozpakuj archiwum do folderu gry.
3. Uruchom grę i wczytaj zapis. V jest teraz widoczna w trzeciej osobie.
4. Opcjonalnie dostosuj mod w **Mod Settings > Ultimate Third Person**.

## Co nowego w 0.2.0

Bazy Motion Matching są takie same jak w 0.1.2. Wersja 0.2.0 zmienia sposób doboru, tempo i łączenie animacji oraz korekty stóp i celowania. Pełną listę zawiera [lista zmian](CHANGELOG.md).

- **Dobór animacji:** działa bliżej pełnego wzorca Motion Matching i ma więcej zabezpieczeń przed migotaniem między animacjami.
- **Tempo animacji:** na ziemi animacje ruchu dopasowują się do rzeczywistej prędkości V, aby ograniczyć ślizganie stóp, a przewidywany ruch, według którego dobierane są animacje, jest kształtowany dla ruszania, zatrzymań, zmian chodu, obrotów, kroków w bok, kucania, celowania, schodów, skoków i lądowań.
- **Dopasowanie kroku:** kroki V są skracane do jej prędkości w pętlach chodu, biegu i ruchu w kucki.
- **Stopy:** postawione stopy mocniej trzymają się ziemi przy obrotach i zatrzymaniach i zachowują swój kierunek, a stopa, na której V obraca się w miejscu, zostaje na miejscu. Stopy są stawiane na zboczach, stopniach, schodach i krawężnikach, a biodra w razie potrzeby się obniżają. To opcje w menu, domyślnie włączone. **Fit steps to stairs** (krótsze, szybsze kroki przy tej samej prędkości) jest opcją dodatkową, domyślnie wyłączoną.
- **Obroty, zatrzymania i uniki:** przy zawracaniu o 180 stopni w ruchu ciało V obraca się w tę samą stronę co animacja obrotu, kolana podążają za kierunkiem bioder, V mniej wybiega do przodu przy zatrzymaniu, a sprint w ścianę utrzymuje pozę bezczynną zamiast migotać między animacjami zwrotów. Kierunek uniku jest liczony względem postaci, a nie obróconej sylwetki, animacja uniku jest obracana w stronę ruchu (maksymalnie o 90 stopni), a korekta kierunku nie skręca już ciała V w jej trakcie.
- **Kucanie:** ruch w kucki odtwarza się w rzeczywistym tempie V; zmienił się też wybór zatrzymania w kucki i ułożenie stóp na jego końcu.
- **Pochylenie:** domyślnie znacznie lżejsze, działa teraz także z wyjętą bronią, przy celowaniu (tylko ciało) i w kucki. W trybie Shooter i podczas obrócenia w stronę celu zależy od bocznego przyspieszenia samej V, więc V nie przechyla się już względem złej osi.
- **Chód i pad:** po włączeniu chodu V porusza się z prędkością 1,9 m/s. **Na padzie zmienia to prędkość poruszania:** domyślnie drążek wychylony mniej niż w około dwóch trzecich daje równy chód, dalej — pełny bieg, a w kucki zawsze używana jest pełna prędkość. Opcja **Follows the stick** przywraca płynną prędkość zależną od wychylenia.
- **Celowanie:** lufa jest korygowana w stronę celownika także na boki, górna część ciała jest stabilniejsza podczas obrotów V i szarpnięć kamery, przy zmianie kierunku w ruchu z celowaniem animacja zwrotu może uruchomić się od mniejszego kąta, a krótkie stuknięcia w lewo i w prawo pozostają przy animacjach zwrotów ze stabilniejszymi stopami. Dłoń podtrzymująca zostaje na broni dwuręcznej i pistolecie, np. przy celowanym ślizgu lub w kucki. Ruch do przodu przy celowaniu jest szybszy (3,85 m/s, wcześniej 3,3), a ruch na boki i w tył jest do niego przeskalowany.
- **Naturalny obrót ciała przy celowaniu w ruchu:** gdy V obróci się już w stronę celownika, jej nogi i biodra zachowują własny kierunek i obracają się razem z krokami, a górna część ciała i broń pozostają na celowniku.
- **Głowa podąża za kamerą:** w trybie Shooter i przy ruchu bokiem bez celowania głowa i szyja V obracają się i pochylają w stronę, w którą patrzy kamera (do 45 stopni w bok), a dalej dołącza górna część ciała.
- **Broń i tryb Shooter:** wyjęcie, schowanie lub zmiana rodzaju broni płynnie przechodzi do nowej pozy bez przestrzelenia; jeśli mod wstrzyma się, by chronić liczbę klatek, gdy V trzyma broń dystansową, poza broni zostaje zachowana. Tryb Shooter (nadal domyślnie wyłączony) utrzymuje każdą animację kierunku przez minimalny czas, aby ograniczyć przeskakiwanie tam i z powrotem.
- **Kamera i windy:** krawężniki i stopnie nie powinny już przełączać kamery w tryb szybkiego opadania, a w windach i na ruchomych platformach V nie powinna już odtwarzać animacji skoku ani spadania; kamera i wysokość ciała powinny poruszać się razem z kabiną.
- **Menu:** każda wartość na listach w Mod Settings powinna mieć teraz czytelną nazwę.

### Nowe i zmienione ustawienia

W **Mod Settings > Ultimate Third Person**:

- **General > Head follows the camera** — nowa, domyślnie włączona.
- **Camera > Walk speed (m/s)** — prędkość po włączeniu chodu; domyślnie 1,9 (wcześniej 1,6).
- **Camera > Stride matching** — nowa, domyślnie włączona.
- **Camera > Firm feet in turns and stops** — nowa, domyślnie włączona.
- **Camera > Feet on uneven ground** — nowa, domyślnie włączona.
- **Camera > Fit steps to stairs** — nowa, domyślnie **wyłączona**; wymaga Feet on uneven ground i Stride matching.
- **Camera > Planted feet keep their direction** — nowa, domyślnie włączona.
- **Camera > Firm pivot foot in turns on the spot** — nowa, domyślnie włączona; wymaga Firm feet in turns and stops.
- **Camera > Natural body turn while aiming on the move** — nowa, domyślnie włączona.
- **Camera > Body lean** i **Head lean** — nowe na tej stronie; domyślnie po 0,2 (wcześniej siła pochylenia wynosiła 0,85 dla ciała i 0,9 dla głowy).
- **Advanced > Gamepad stick speed** — nowa: Walk or run (domyślnie) albo Follows the stick (prędkość zmienia się płynnie z wychyleniem drążka, jak w podstawowej grze).

Wartości domyślne pozostałych opcji na tej stronie się nie zmieniły.

## Zobacz różnicę

Po lewej: oryginalne animacje gry. Po prawej: Ultimate Third Person Project. Nagrania w rzeczywistym tempie, wykonane na wcześniejszej wersji moda.

| Bieg, ślizg, kucanie | Celowanie i strzelanie | Pięści |
|:---:|:---:|:---:|
| <img src="docs/media/movement.gif" width="280"> | <img src="docs/media/aiming.gif" width="280"> | <img src="docs/media/fists.gif" width="280"> |

## Funkcje

| | | |
|:---|:---|:---|
| 🏃 **Motion Matching**<br>Animacje ruchu dobierane klatka po klatce z dużej bazy. | 🔄 **Obroty ciała**<br>V naturalnie obraca się wraz z kamerą i kierunkiem ruchu. | 🦶 **Stawianie stóp**<br>Blokowanie stóp oraz opcje stabilnych stóp, dopasowania kroku i nierównego terenu. |
| 🎯 **Celowanie**<br>Najbardziej responsywne celowanie i strzelanie w trzeciej osobie w grze. | 🔫 **Odrzut i chwyt broni**<br>Odrzut z animacji broni w grze, lewa dłoń na łożu, kolba przy ramieniu. | 🥊 **Walka wręcz**<br>Pięści, noże, katany, broń obuchowa, pałki, Ostrza Modliszkowe i Ręce Goryla w trzeciej osobie, oparte na oryginalnych animacjach walki z gry. |
| 🚗 **Pojazdy**<br>Wsiadanie do samochodów i na motocykle oraz wysiadanie w trzeciej osobie. | ↔️ **Zmiana ramienia kamery**<br>Jeden klawisz na klawiaturze i padzie, przypisywany w Mod Settings. | 🎞️ **Pakiet animacji**<br>Animacje UTPP lub oryginalne animacje gry, przełączane w menu. |
| 🩻 **Poprawki deformacji**<br>Skóra ciała przeliczana dla póz moda oraz dodatkowe korekty ramion i przedramion. | ✨ **Płynne przejścia**<br>Mniej nagłych przeskoków między animacjami. | 🧾 **Logi diagnostyczne**<br>Wbudowane logi i plik raportu ułatwiają zgłaszanie problemów. |

<p align="center"><img src="docs/media/divider.svg" width="100%"></p>

### Obecne funkcje (prace w toku)

- Lokomocja oparta na Motion Matching
- Płynne przejścia między sprintem, zatrzymaniem, obrotem i kucaniem
- Obroty ciała podczas ruchu
- Koordynacja kierunku kamery i ruchu
- Blokowanie stóp ograniczające widoczne ślizganie, z opcjami stabilnych postawionych stóp i stabilnej stopy obrotu przy zwrotach i zatrzymaniach
- Dopasowanie kroku: kroki skracane do prędkości w chodzie, biegu i ruchu w kucki (opcja w menu)
- Stopy stawiane na zboczach, stopniach, schodach i krawężnikach (opcja w menu); dopasowanie kroków do schodów trzeba włączyć
- Animacje ruchu odtwarzane na ziemi w rzeczywistym tempie V
- Głowa podąża za kamerą w trybie Shooter i przy ruchu bokiem bez celowania
- Regulowane pochylenie ciała i głowy w zakrętach
- Najbardziej responsywne celowanie i strzelanie w trzeciej osobie, jakie dotąd powstało dla Cyberpunk 2077
- Ulepszony odrzut i mieszanie animacji górnej części ciała
- Lepsza ciągłość ułożenia dłoni, broni i przeładowania, w tym chwyt łoża lewą dłonią i kolba przy ramieniu
- Walka wręcz w trzeciej osobie oparta na oryginalnych animacjach gry: pięści, noże, katany, jednoręczna i dwuręczna broń obuchowa, pałki, Ostrza Modliszkowe i Ręce Goryla
- Pozycja gardy między ciosami, ciosy poruszające całym ciałem podczas stania i pozy walki wręcz w kucki
- Pięści: podczas uderzenia V obraca się w stronę przeciwnika śledzonego przez grę
- Walka dwiema broniami (pierwsza wersja): pozy, celowanie i odrzut w trzeciej osobie dla dwóch broni jednoręcznych
- Uniki na ziemi z własnymi animacjami, w kierunku wskazanym przez gracza
- Wsiadanie do samochodów i na motocykle oraz wysiadanie w trzeciej osobie
- Klawisz zmiany ramienia kamery (klawiatura i pad, przypisywany w Mod Settings)
- Ruch głowy: V obraca głowę i oczy w kierunku, w którym patrzy kamera, podczas stania lub chodzenia
- Skóra obu wariantów ciała V przeliczana pod pozy moda, aby ograniczyć deformacje
- Bardziej kobieca sylwetka V z kobiecym ciałem, gdy nie trzyma broni: opadające, cofnięte barki i ręce bliżej ciała (można wyłączyć)
- Cień V podąża za ciałem w trzeciej osobie, także przy cieniach ray tracing
- Ukierunkowane korekty deformacji ramion i przedramion
- Ograniczone nagłe przeskoki między animacjami
- Wybór animacji: animacje UTPP albo oryginalne animacje gry z zachowaniem kamery w trzeciej osobie i widocznego ciała
- Wbudowane logi diagnostyczne ułatwiające zgłaszanie problemów

UTPP łączy lokomocję w trzeciej osobie, Motion Matching, obroty ciała, koordynację kamery, celowanie, odrzut, obsługę broni i przejścia animacji w jeden powiązany system.

**Celem UTPP jest stworzenie możliwie najbardziej kompletnego doświadczenia trzecioosobowego w Cyberpunk 2077,** w którym ruch, animacje, kamera i walka współpracują naturalnie.

Docelowo projekt ma obsługiwać pełne animacje fabularne, lokomocję Motion Matching i wszystkie główne systemy walki, w tym broń palną, walkę wręcz, ostrza, pięści, cyberwszczepy oraz pozostałe interakcje bojowe.

Celem nie jest jedynie pokazanie walki zza pleców V, lecz sprawienie, by każdy system walki był odpowiednio zaprojektowany pod perspektywę trzecioosobową.

**UTPP jest nadal aktywnie rozwijany.** Wiele póz, broni, stanów ciała i przejść wymaga dalszego dopracowania, ale fundament już działa i jest stale ulepszany.

**Jeśli chcesz pomóc w rozwoju Ultimate Third Person Project dla Cyberpunk 2077, pobierz UTPP, sprawdź go w swojej konfiguracji i podziel się uwagami.**

<p align="center"><img src="docs/media/divider.svg" width="100%"></p>

## Pakiet animacji

Wybierz animacje w **Mod Settings > Ultimate Third Person > General > Animation pack**.

- **Ultimate Third Person Project** (domyślnie) — animacje tego moda dla ruchu, celowania, przeładowania, walki wręcz i pojazdów.
- **Game Animations** — oryginalne animacje gry na ciele widocznym w trzeciej osobie. Kamera trzecioosobowa, całe ciało, głowa, celownik i klawisz zmiany ramienia pozostają aktywne. Ruch moda, pozy broni, odrzut, chwyt dłoni, korekty barków i pochylenia, pokonywanie przeszkód, autorskie przeładowanie, animacje walki wręcz, wsiadania i wysiadania oraz tryb Shooter zostają wyłączone, a zależne od nich opcje nie mają działania. V obraca się wraz z kamerą, tak jak w grze.

Zmiana działa od razu, gdy V stoi bezczynnie. Animacje walki wręcz, pojazdów i przeładowania zmieniają się przy następnym wyjęciu broni, wejściu do pojazdu lub przeładowaniu. Gdy V przeładowuje, celuje, walczy, wspina się, prowadzi pojazd lub uczestniczy w scenie, zmiana czeka do zakończenia tej czynności. Pozostałe ustawienia są zachowane i wracają po ponownym wybraniu animacji UTPP. Archiwa moda pozostają zainstalowane i nadal zastępują część zasobów gry także z tym pakietem; ich wpisy UTPP są wtedy nieaktywne.

Oryginalne animacje gry powstały dla ciała pierwszoosobowego, dlatego przy Game Animations mogą wrócić deformacje ramion i barków — to właśnie je korygują animacje UTPP.

## Wymagania

Poniższe wymagania są potrzebne do poprawnego działania UTPP.

**Wymagana wersja gry**

- **Cyberpunk 2077 w wersji 2.5**
- **Dodatek Phantom Liberty**

> [!IMPORTANT]
> UTPP jest przeznaczony do gry w wersji 2.5 z Phantom Liberty. Starsze wersje gry nie są obsługiwane.

**Wymagane frameworki i mody pomocnicze**

- **RED4ext 1.31.1** — ładuje główny plugin UTPP.
- **redscript 0.5.32** — wymagany przez skrypty UTPP.
- **Cyber Engine Tweaks 1.38.1** — udostępnia logikę ustawień, kamery, ciała i pojazdów wykorzystywaną przez UTPP.
- **Codeware 1.21.4** — udostępnia rozszerzenia API używane przez skrypty UTPP.
- **ArchiveXL 1.28.8** — zapewnia dodatkową obsługę animacji i archiwów wykorzystywaną przez UTPP.
- **Input Loader** (wersja 0.2.6) — zapewnia klawisz zmiany ramienia kamery; pozostałe funkcje działają bez niego.
- **Mod Settings** (zalecany, wersja 0.2.30) — strona Ultimate Third Person w ustawieniach gry.

> [!WARNING]
> UTPP 0.2.0 jest przeznaczony dla wersji podanych powyżej; wymagania są takie same jak w 0.1.2. Starsze nie będą działać z grą w wersji 2.5. Nowsze mogą działać, ale ich zgodność nie jest gwarantowana do czasu sprawdzenia.

Zainstaluj i zaktualizuj wymagane frameworki przed zgłoszeniem problemu z UTPP.

Nie używaj przestarzałych, zdublowanych ani wymieszanych wersji wymaganych frameworków. Jeśli gra nie uruchamia się lub UTPP nie zostaje załadowany, najpierw sprawdź poprawność instalacji zależności i ich zgodność z wersją gry.

## Instalacja

<details>
<summary><b>Instalacja przez Vortex</b></summary>

1. Otwórz sekcję Files na [stronie UTPP w Nexus Mods](https://www.nexusmods.com/cyberpunk2077/mods/35614).
2. Kliknij **Mod manager download**.
3. Zainstaluj i włącz UTPP w Vortex.
4. Wdróż mody, jeśli Vortex o to poprosi.
5. Uruchom Cyberpunk 2077 i wczytaj zapis.
6. Miłej zabawy!

</details>

<details>
<summary><b>Instalacja ręczna</b></summary>

1. Otwórz sekcję Files na [stronie UTPP w Nexus Mods](https://www.nexusmods.com/cyberpunk2077/mods/35614).
2. Kliknij **Manual download**.
3. Otwórz pobrane archiwum.
4. Wypakuj zawartość pobranego folderu do katalogu gry Cyberpunk 2077.
5. Zachowaj strukturę folderów z archiwum.
6. Uruchom Cyberpunk 2077 i wczytaj zapis.
7. Miłej zabawy!

</details>

> [!CAUTION]
> Przy aktualizacji UTPP usuwaj stare pliki wyłącznie wtedy, gdy nakazuje to opis wydania. Nie mieszaj plików z różnych wersji UTPP.

**Aktualizacja do 0.2.0:** zainstaluj 0.2.0 na posiadaną wersję i pozwól zastąpić wszystkie pliki. Nie trzeba usuwać żadnego pliku ze starszej wersji. Przy przejściu z dowolnej wersji 0.1.x ustawienia moda zostaną przy pierwszym uruchomieniu jednorazowo przywrócone do domyślnych, ponieważ 0.2.0 zmienia ruch i jego wartości domyślne; poinformuje o tym komunikat na ekranie. Świeża instalacja nie wyświetla komunikatu. Dwa przypisania „Swap shoulder” pozostaną bez zmian. Dzieje się to tylko raz — później ustawienia są zachowywane jak wcześniej.

Przed instalacją lub aktualizacją UTPP zawsze zachowaj kopię swojej konfiguracji modów.

## Rozwiązywanie problemów

> [!TIP]
> Zmień **Animation pack** na **Game Animations**, aby sprawdzić, czy problem wynika z animacji UTPP, czy z czegoś innego.

<details>
<summary><b>Jeśli UTPP nie działa prawidłowo</b></summary>

- Sprawdź, czy Cyberpunk 2077 ma obsługiwaną wersję.
- Upewnij się, że Phantom Liberty jest zainstalowane.
- Sprawdź, czy wszystkie wymagane frameworki są zainstalowane i włączone.
- Upewnij się, że Vortex wdrożył zainstalowane mody.
- Sprawdź, czy nie pozostały pliki starszych wersji UTPP.
- Po instalacji lub aktualizacji zależności uruchom grę ponownie.
- Przy podejrzeniu konfliktu sprawdź UTPP bez innych modów zmieniających trzecią osobę, animacje lub walkę.

Jeśli po aktualizacji gra się zawiesza lub zachowuje nietypowo, tymczasowo wyłącz UTPP i sprawdź, czy wymagane frameworki działają samodzielnie, zanim ponownie zainstalujesz mod.

</details>

## Jak zgłosić problem

<details>
<summary><b>Korzystanie z wbudowanych logów diagnostycznych</b></summary>

1. Otwórz **Mod Settings > Ultimate Third Person > General > Diagnostic logs** i wybierz **On for 5 minutes**. Komunikat na ekranie poinformuje o włączeniu logów.
2. Odtwórz problem w grze. Logi zbierają dane przez 5 minut rozgrywki (czas nie płynie w menu ani podczas pauzy), a następnie wyłączają się automatycznie. Możesz też wcześniej ustawić opcję na **Off**. Komunikat poinformuje o wyłączeniu logów i poda nazwę pliku raportu.
3. Otwórz folder `red4ext\logs` w katalogu gry. Plik `utpp-report-(date).txt` zawiera listę plików do wysłania. Spakuj je do ZIP-a i wyślij z krótkim opisem tego, co się wydarzyło.

Logi nie zawierają danych osobowych: ścieżki są zapisywane względem folderu gry, bez nazwy Twojego konta Windows.

**On until switched off** pozostawia logi włączone, także po ponownym uruchomieniu gry. Używaj tej opcji wyłącznie do długich testów i później ją wyłącz; nagrywanie póz zachowuje wtedy ostatnie 10 minut.

**Przy zgłaszaniu problemu podaj również:**

- Wersję Cyberpunk 2077
- Wersję UTPP
- Wersje zainstalowanych frameworków
- Konfigurację postaci i broni
- Stan ruchu lub walki, w którym pojawił się problem
- Zrzut ekranu lub nagranie, jeśli to możliwe

Czytelne i odtwarzalne zgłoszenia są szczególnie pomocne, gdy problem występuje tylko dla określonej broni, pozy, stanu ruchu lub zestawu modów.

</details>

## Zgodność z innymi modami

<details>
<summary><b>Szczegóły</b></summary>

UTPP może oddziaływać z modami zmieniającymi:

- Działanie kamery
- Lokomocję gracza
- Grafy animacji
- Celowanie i strzelanie
- Odrzut
- Obsługę broni
- Animacje przeładowania
- Deformacje ciała
- Przejścia między stanami walki

Nie można zagwarantować zgodności z każdym modem. Jeśli inny mod zmienia te same systemy co UTPP, tymczasowo go wyłącz i ponów test przed zgłoszeniem problemu.

UTPP zastępuje trzy grafy animacji podstawowej gry: `player_melee.animgraph` (animacja po sparowaniu ciosu dobierana dla każdego ataku; plik `utpp_melee_parried_graph.archive`, nowy w 0.1.1) oraz `player_ranged.animgraph` i `weapon.animgraph` (opcjonalne autorskie przeładowanie pistoletu Nue; plik `utpp_nue_reload.archive`). Mody zastępujące te same grafy kolidują z tymi plikami.

Na potrzeby diagnozy sprawdzaj UTPP z wymaganymi zależnościami, bez dodatkowych modów zmieniających trzecią osobę, animacje lub walkę. Po potwierdzeniu działania UTPP włączaj inne mody pojedynczo, aby znaleźć ewentualne konflikty.

</details>

## Znane ograniczenia

<details>
<summary><b>Szczegóły</b></summary>

Wersja 0.2.0 to czwarte publiczne wydanie UTPP i nadal wczesny etap projektu.

Niektóre pozy, rodzaje broni, stany ciała, animacje fabularne i przejścia wymagają dalszego dopracowania. Efekt może też zależeć od konfiguracji postaci, broni i animacji.

Nie wszystkie planowane funkcje i systemy walki zostały już w pełni wdrożone.

Nadal mogą wystąpić:

- Deformacje w określonych pozach
- Przejścia wymagające dalszego wygładzenia
- Różnice między staniem, kucaniem i ruchem
- Ograniczenia dotyczące konkretnych rodzajów broni
- Broń dwuręczna: górna część ciała może nieznacznie pochylić się i przesunąć na początku lub końcu celowania
- Zmiany działania animacji spowodowane innymi modami
- Deformacje ramion i barków przy użyciu opcji Game Animations
- Funkcje planowane na kolejne wydania, ale jeszcze niewdrożone w pełni (np. młoty, monostruna, finiszery i obalenia, które zachowują oryginalne animacje gry)

Projekt rozwija się etapami, ze stałym naciskiem na jakość ruchu, ciągłość animacji, deformacje ciała i integrację walki.

</details>

## Znane problemy w 0.2.0

<details>
<summary><b>Szczegóły</b></summary>

Te problemy są znane w wersji 0.2.0 i planowane do poprawy w kolejnych aktualizacjach. Nie musisz ich zgłaszać. Zmiany od wersji 0.1.2 opisano na [liście zmian](CHANGELOG.md). Lista obejmuje znane problemy z 0.1.2, z których nie wszystkie zostały ponownie sprawdzone.

- Wspinanie lub przeskok rozpoczęte tuż po skoku często nie zostaje wykonane
- Celowanie: przy szarpnięciach kamery i szybkich obrotach lufa może nieco bardziej zostawać za celownikiem, ponieważ celowanie jest wygładzane
- Głowa podążająca za kamerą: szyja V może się bardziej deformować, głównie u V z męskim ciałem (opcję można wyłączyć); pochylenie działające w większej liczbie stanów nieznacznie to powiększa
- Stopy: przy schodzeniu po schodach kolano może w pojedynczych klatkach mocno się zgiąć; przy ruszaniu V może częściej zmieniać nogę prowadzącą; przy części obrotów i krótkich stuknięciach w bok podczas celowania mogą pojawić się większe korekty lub ślizganie stóp, głównie u V z męskim ciałem
- Stabilne stopy przy zwrotach i zatrzymaniach: obrót V z męskim ciałem w biegu może wyglądać nieco gorzej
- Naturalny obrót ciała przy celowaniu w ruchu: przy niektórych ruszeniach stopy mogą bardziej się ślizgać, a część dużych zmian kierunku wypada gorzej
- Rzadki crash kilka sekund po wczytaniu zapisu, w którym V siedzi w pojeździe; ponowne wczytanie działa
- Nóż: gdy kolejny cios kombinacji zaczyna się późno podczas chodzenia, ramiona mogą gwałtownie przeskoczyć między ciosami
- Ostrza Modliszkowe i Ręce Goryla: wyjęcie odtwarza oryginalną animację gry
- Młoty, monostruna, finiszery i obalenia zachowują oryginalne animacje gry
- Walka dwiema broniami jest w pierwszej wersji: przeładowanie korzysta z podstawowej animacji, a postawa wymaga dopracowania
- Kobieca sylwetka: widać ją, gdy V z kobiecym ciałem stoi i porusza się bez broni. Z wyjętą bronią, podczas wspinania i przeskoków oraz w animacjach odtwarzanych przez samą grę używana jest poprzednia sylwetka; przejście między nimi trwa ćwierć sekundy
- Karabiny snajperskie: celowanie przez lunetę korzysta z widoku pierwszoosobowego; po zakończeniu celowania ten widok utrzymuje się jeszcze przez około sekundę, a kamera wraca do trzeciej osoby przez ciało V
- Broń dwuręczna: V mocno pochyla się do przodu w ruchu, a kolby niektórych karabinów nie sięgają ramienia
- Strzelby, karabiny snajperskie i precyzyjne oraz ciężkie karabiny maszynowe nie mają jeszcze animacji odrzutu UTPP
- Przeładowanie: w niektórych animacjach karabinów łokieć ręki podtrzymującej broń może gwałtownie przeskoczyć na jedną klatkę
- Ruch głowy: gdy kamera jest daleko za ramieniem V, głowa obraca się blisko granicy zakresu i może wyglądać sztywno
- Dash podczas sprintu korzysta z krótkich animacji uników i może wyglądać sztywno
- Stopnie terenu wyższe niż około 30 cm mogą na chwilę przerwać bieg
- Przeskok lub ślizg przez maskę samochodu z wyciągniętą bronią: górna część ciała może odchylić się do tyłu
- Pojazdy: przy wysiadaniu z niektórych samochodów dolna część nogi może wejść w próg, a poza na chwilę się przesunąć
- Motocykl w kamerze pierwszoosobowej: ramiona i tułów mogą wyglądać na zniekształcone
- Sceny i dialogi domyślnie pozostają w trzeciej osobie i odtwarzają oryginalne animacje gry przeznaczone do widoku pierwszoosobowego: w części z nich ciało V może być zniekształcone. Włącz „First person in scenes and dialogues”, aby oglądać je w pierwszej osobie

</details>

## Podziękowania

Szczególne podziękowania dla społeczności modderskiej Cyberpunk 2077 i wszystkich twórców narzędzi, dokumentacji, analiz oraz zasobów, dzięki którym projekty takie jak UTPP mogą powstawać.

Dziękuję twórcom licencjonowanych animacji i materiałów technicznych, na których opiera się UTPP (zobacz [Licencję i credits](#licencja-i-credits)).

**Dziękuję każdemu, kto pobiera UTPP, testuje go, zgłasza błędy, dzieli się uwagami lub wspiera projekt. Wasze wsparcie pomaga kontynuować rozwój.**

<p align="center"><img src="docs/media/divider.svg" width="100%"></p>

# Dla programistów

## Struktura repozytorium

- `plugin/UTPP/` — sam mod.
  - `src/` — plugin RED4ext (`UTPP.dll`, C++20, Windows/MSVC).
  - `scripts/` — skrypty gry w redscript (`.reds`).
  - `cet/` — mody Cyber Engine Tweaks (Lua).
  - `archive/`, `archive-src/`, `input/` — zasoby ArchiveXL i definicja klawisza Input Loader.
  - `tests/` — testy offline logiki pluginu.
- `tools/` — narzędzia Python (z pomocnikami PowerShell) do przetwarzania animacji, przygotowania danych, testów, instalacji i pakowania wydań.
- `ue/` — skrypty eksportu z Unreal Engine.

## Wykorzystanie AI

<details>
<summary><b>Szczegóły</b></summary>

Projekt UTPP jest opracowywany i kierowany przez autora. Przy jego tworzeniu korzystano z generatywnej AI:

- Kod: większość pluginu, skryptów i narzędzi napisali agenci programistyczni AI pod kierunkiem autora
- Animacje: część przejść, wysiadania z pojazdów i animacji walki wręcz wygenerowano lokalnie przy użyciu NVIDIA Kimodo i MotionBricks
- Strona moda: logo, zwiastun i opis strony powstały z pomocą AI
- W samym modzie nie wykorzystano głosów, tekstur ani tłumaczeń AI. Polskie README przetłumaczono z pomocą AI.
- Nic w paczce nie pochodzi z innych modów ani nie zostało z nich opracowane bez zgody lub licencji

</details>

## Licencja i credits

Ultimate Third Person Project jest objęty prawami autorskimi (c) Daniel Maciej Pytel. Wszelkie prawa zastrzeżone, z wyjątkiem zakresu objętego licencjami zewnętrznymi.

- **Dane animacji.** Dostarczane z UTPP bazy Motion Matching, nakładki, tabele pokonywania przeszkód i pochylenia są autorską pracą. Oparto je na licencjonowanych zasobach GASPALS 1.2 autorstwa Polygon Hive, a następnie znacznie zmodyfikowano, rozszerzono, dostrojono i połączono z oryginalnymi animacjami Cyberpunk 2077. Zasoby GASPALS wykorzystano na podstawie zakupionej licencji.
- **Wygenerowane animacje.** Część przejść, wysiadania z pojazdów i animacji walki wręcz wygenerowano lokalnie przy użyciu NVIDIA Kimodo i NVIDIA MotionBricks (modele na licencjach odpowiednio NVIDIA Open Model License i NVIDIA Open Model Agreement, udzielanych przez NVIDIA Corporation). Żaden model NVIDIA nie jest dołączony.
- **Materiały pochodzące z Cyberpunk 2077.** Profile odrzutu broni i animacje, grafy oraz poprawki zasobów opracowane z animacji i zasobów Cyberpunk 2077 oraz dodatku Phantom Liberty (c) CD PROJEKT RED albo na ich potrzeby wywodzą się z zawartości gry. Nie zawierają gry i działają wyłącznie z jej legalną kopią.
- **Kod open source (MIT)** używany w `UTPP.dll`: RED4ext.SDK (c) 2020 – obecnie Octavian Dima (nagłówki API pluginu); cyberpunk-vr-port (c) 2026 dariulone (zaadaptowano logikę i offsety pamięci do uruchamiania animacji gracza); Motion-Matching (c) 2021 Daniel Holden (ponownie zaimplementowane algorytmy inercjalizacji i Motion Matching). Pełne teksty licencji znajdują się w [`THIRD_PARTY_NOTICES.txt`](THIRD_PARTY_NOTICES.txt) i są dołączane do każdego wydania.

Projekt nie jest powiązany z CD PROJEKT RED, Epic Games, Polygon Hive ani NVIDIA i nie jest przez te firmy oficjalnie wspierany.

## Autor

Daniel Maciej Pytel
