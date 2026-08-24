# Merenje temperature u automobilu

## Opis projekta

Ovaj projekat predstavlja softversku simulaciju sistema za merenje unutrašnje i spoljašnje temperature u automobilu, realizovanu korišćenjem FreeRTOS real-time operativnog sistema i hardverskog simulatora.

Za merenje se koriste dva senzorska kanala:

- **Kanal 0** – unutrašnji senzor temperature
- **Kanal 1** – spoljašnji senzor temperature
- **Kanal 2** – komunikacija sa PC-jem

Senzori šalju vrednosti otpornosti u opsegu od **0 do 71 Ω**. Sistem prima vrednosti preko serijske komunikacije, računa prosek poslednjih pet merenja i na osnovu prosečne otpornosti izračunava temperaturu.

Dobijene temperature se prikazuju na LED bar indikatorima i 4-cifrenom 7-segmentnom displeju, dok se preko PC kanala mogu podešavati kalibracioni parametri i temperaturne granice za alarm.

---

## Funkcionalnosti

Sistem podržava sljedeće funkcionalnosti:

1. Automatsko okidanje oba senzora svakih **1 s**.
2. Prijem vrednosti otpornosti preko serijskih kanala 0 i 1.
3. Obradu dvocifrenih vrednosti otpornosti u opsegu **0–71 Ω**.
4. Čuvanje poslednjih 5 merenja za svaki senzor.
5. Računanje prosečne otpornosti.
6. Pretvaranje otpornosti u temperaturu pomoću linearne kalibracije.
7. Prikaz unutrašnje i spoljašnje temperature na LED barovima.
8. Alarmnu signalizaciju kada temperatura izađe iz podešenih granica.
9. Ciklično prikazivanje otpornosti i temperature na 7-segmentnom displeju.
10. Komunikaciju sa PC-jem preko UniCom kanala 2.
11. Podešavanje temperaturnih parametara pomoću PC komandi.
12. Periodično slanje trenutnih temperatura prema PC-ju.

---

## Način rada sistema

Sistem se izvršava kroz više FreeRTOS taskova.

Proces merenja odvija se sledećim redosledom:

1. `SensorTrigger_Task` svakih 1 s šalje trigger karaktere `X`, `Y` i `Z` prema oba senzora.
2. Senzori vraćaju vrednost otpornosti preko kanala 0 i 1.
3. `SerialReceive_Task` prima serijske karaktere i formira vrednost otpornosti.
4. Vrednosti u opsegu od 0 do 71 Ω šalju se u `Sensor_Queue`.
5. `TemperatureProcess_Task` čuva poslednjih pet merenja za odgovarajući senzor.
6. Računa se prosečna otpornost.
7. Prosečna otpornost se pretvara u temperaturu pomoću linearne kalibracije.
8. Izračunate temperature šalju se u `Temperature_Queue`.
9. `LEDBar_Task` prikazuje unutrašnju i spoljašnju temperaturu na LED barovima i kontroliše alarm.
10. Podaci se šalju u `LCD_Queue` za prikaz na 7-segmentnom displeju.
11. `TemperatureDisplay_Task` svakih 1 s formira izveštaj o temperaturama.
12. `PCSend_Task` šalje izveštaj preko PC kanala.

---

## Proračun temperature

Maksimalna vrednost otpornosti senzora je:

```text
Rmax = 71 Ω

```

Početne kalibracione vrednosti su:

```text
Tmin = 20 °C
Tmax = 50 °C

```

Temperatura se računa linearnom interpolacijom:

```text
T = Tmax - R × (Tmax - Tmin) / Rmax

```

Za početne vrednosti sistema formula je:

```text
T = 50 - R × 30 / 71

```

Kalibracione vrednosti `Tmin` i `Tmax` mogu se promeniti pomoću PC komandi.

---

## FreeRTOS arhitektura

### Taskovi

| TaskOpis                  |                                              |
| ------------------------- | -------------------------------------------- |
| `SensorTrigger_Task`      | Šalje trigger karaktere senzorima svakih 1 s |
| `SerialReceive_Task`      | Prima i obrađuje podatke sa senzora          |
| `TemperatureProcess_Task` | Računa prosečnu otpornost i temperaturu      |
| `LEDBar_Task`             | Prikazuje temperature i upravlja alarmom     |
| `PCReceive_Task`          | Prima i obrađuje PC komande                  |
| `TemperatureDisplay_Task` | Formira izveštaj o temperaturama             |
| `PCSend_Task`             | Šalje poruke prema PC-ju                     |
| `LCDDisplay_Task`         | Upravlja 4-cifrenim 7-segmentnim displejem   |

### Redovi

Koriste se sledeći FreeRTOS redovi:

- `Sensor_Queue` – prenosi podatke o otpornosti senzora;
- `Temperature_Queue` – prenosi izračunate temperature;
- `PCSend_Queue` – prenosi poruke namenjene PC-ju;
- `LCD_Queue` – čuva najnovije podatke za prikaz na 7-segmentnom displeju.

### Semafori

Za sinhronizaciju serijske komunikacije i prekida koriste se binarni semafori za:

- prijem podataka (`RXC`);
- završetak slanja podataka (`TBE`);
- LED interrupt;
- osvežavanje 7-segmentnog displeja.

### Tajmer

`LCD_Timer` je FreeRTOS softverski tajmer sa periodom od **100 ms**.

Tajmer daje semafor `LCDDisplay_Task` tasku, čime se omogućava periodično osvežavanje displeja.

---

## LED bar prikaz

LED barovi se koriste za prikaz temperatura i alarmnog stanja.

- **LED bar 1** – unutrašnja temperatura;
- **LED bar 2** – spoljašnja temperatura;
- **LED bar 0** – alarm.

Broj aktivnih LED dioda određuje se funkcijom `TemperatureToLEDPattern()`.

Prikaz temperature koristi sledeće intervale:

| TemperaturaBroj LED dioda |   |
| ------------------------- | - |
| < 10 °C                   | 0 |
| 10–19 °C                  | 1 |
| 20–29 °C                  | 2 |
| 30–39 °C                  | 3 |
| 40–49 °C                  | 4 |
| 50–59 °C                  | 5 |
| 60–69 °C                  | 6 |
| 70–79 °C                  | 7 |
| ≥ 80 °C                   | 8 |

Ako je unutrašnja ili spoljašnja temperatura izvan podešenog intervala `[TLOW, THIGH]`, aktivira se alarmni LED bar.

Alarmni LED bar se uključuje i isključuje u periodama od **500 ms**.

---

## 7-segmentni displej

4-cifreni 7-segmentni displej prikazuje podatke ciklično.

Podržani prikazi su:

| OznakaPodatak |                        |
| ------------- | ---------------------- |
| `Ir`          | unutrašnja otpornost   |
| `Or`          | spoljašnja otpornost   |
| `It`          | unutrašnja temperatura |
| `Ot`          | spoljašnja temperatura |

Displej se osvežava svakih **100 ms**.

Prikaz se menja nakon 10 osvežavanja, odnosno svake **1 s**:

```text
Ir → Or → It → Ot → Ir → ...

```

Za numeričke vrednosti prikazuju se dve cifre.

---

## PC komunikacija

PC komunikacija se odvija preko **UniCom kanala 2**.

Komande se završavaju karakterom `CR` (`\r`), a sistem prihvata i `LF` (`\n`).

### `MINTEMP`

Podešava minimalnu temperaturu kalibracionog opsega.

Primer:

```text
MINTEMP20\r

```

Nakon uspešne obrade sistem šalje:

```text
OK\r\n

```

### `MAXTEMP`

Podešava maksimalnu temperaturu kalibracionog opsega.

Primer:

```text
MAXTEMP50\r

```

Odgovor:

```text
OK\r\n

```

### `THIGH`

Podešava gornju granicu temperature za alarm.

Primer:

```text
THIGH80\r

```

Odgovor:

```text
OK\r\n

```

### `TLOW`

Podešava donju granicu temperature za alarm.

Primer:

```text
TLOW10\r

```

Odgovor:

```text
OK\r\n

```

---

## Izveštaj prema PC-ju

Sistem svakih **1 s** formira poruku sa trenutnim temperaturama.

Format poruke je:

```text
TIN=<unutrašnja temperatura> C TOUT=<spoljašnja temperatura> C

```

Primer:

```text
TIN=42 C TOUT=35 C

```

Poruka se šalje preko UniCom kanala 2.

---

## Pokretanje projekta

Za pokretanje projekta potrebno je:

1. Otvoriti razvojno okruženje projekta.
2. Uključiti `main_application.c`.
3. Pokrenuti hardverski simulator.
4. Pokrenuti UniCom simulatore.
5. Konfigurisati komunikacione kanale:

```text
Kanal 0 → unutrašnji senzor
Kanal 1 → spoljašnji senzor
Kanal 2 → PC komunikacija

```

6. Podesiti simulatore senzora tako da nakon trigger karaktera vraćaju vrednosti otpornosti od 0 do 71 Ω.
7. Kompajlirati projekat.
8. Pokrenuti `main_demo`.

Nakon uspešnog pokretanja scheduler-a sistem počinje sa periodičnim merenjem i obradom podataka.

---

## Testiranje

### Test 1 – Senzori

Na kanalima 0 i 1 podesiti automatski odgovor na trigger.

Za test koristiti, na primer:

```text
20

```

Vrednost predstavlja otpornost od:

```text
R = 20 Ω

```

Sistem treba da prikaže obrađenu prosečnu otpornost i izračunatu temperaturu.

Za početne kalibracione vrednosti:

```text
T = 50 - 20 × 30 / 71

```

što daje približno:

```text
T = 42 °C

```

---

### Test 2 – PC komande

Na kanalu 2 poslati:

```text
MINTEMP20\r
MAXTEMP50\r
THIGH80\r
TLOW10\r

```

Za svaku validnu komandu očekuje se odgovor:

```text
OK\r\n

```

---

### Test 3 – Alarm

Podesiti granice tako da trenutna temperatura bude izvan dozvoljenog opsega.

Na primer:

```text
TLOW10\r
THIGH20\r

```

Ako je temperatura veća od 20 °C ili manja od 10 °C, aktivira se alarmni LED bar.

Alarm treba da treperi sa periodom od 500 ms.

---

### Test 4 – 7-segmentni displej

Proveriti da se nakon pristizanja senzorskih podataka prikazi menjaju redom:

```text
Ir → Or → It → Ot

```

Svaki prikaz traje približno 1 s, dok se sam displej osvežava svakih 100 ms.

---

### Test 5 – Pokretni prosek

Poslati više različitih vrednosti otpornosti.

Sistem čuva najviše poslednjih pet merenja za svaki senzor i računa njihov prosek.

Nakon pet merenja, nova merenja zamenjuju najstarija merenja pomoću kružnog bafera.

---

## Struktura projekta

Glavni aplikacioni modul je:

```text
main_application.c

```

U njemu su implementirani:

```text
main_demo()

SensorTrigger_Task
SerialReceive_Task
TemperatureProcess_Task
LEDBar_Task
PCReceive_Task
TemperatureDisplay_Task
PCSend_Task
LCDDisplay_Task

CalculateTemperature()
TemperatureToLEDPattern()
ParseTemperatureValue()
CalculateAverage()
LCDTimerCallback()

```

Projekat koristi FreeRTOS za upravljanje taskovima i sinhronizaciju, kao i hardverski simulator preko dostavljenog `HW_access.h` interfejsa.

---

## MISRA C:2012 analiza i odstupanja

Aplikativni modul `main_application.c` analiziran je alatom **PC-lint 9.00L** prema MISRA C:2012 pravilima obrađenim na vežbama.

Kod je usklađen gdje je to moguće bez izmene FreeRTOS jezgra, Windows/MSVC zaglavlja, hardverskog simulatora i dostavljenog `HW_access.h`.

Tokom analize izvršene su sledeće izmene:

- funkcije i podaci koji pripadaju samo aplikativnom modulu označeni su kao `static`;
- korišćeni su tipovi određene širine iz `stdint.h`;
- uklonjen je nekorišćen kod;
- složeni uslovi imaju eksplicitne zagrade;
- dodati su završni `else` blokovi gde je potrebno;
- proveravaju se povratne vrednosti FreeRTOS API funkcija i hardverskih funkcija;
- formatiranje PC poruke ograničeno je veličinom bafera;
- rezultat formatiranja poruke se proverava;
- scheduler se pokreće samo nakon uspešne inicijalizacije potrebnih resursa.

### Završni status

Aplikativni kod nema prijave za:

- **Rule 17.7**
- **Directive 4.7**
- **Rule 10.8**
- **Rule 15.4**

### Dokumentovana odstupanja

Preostale prijave odnose se na dostavljene biblioteke, FreeRTOS port i platformu:

- **Rule 10.3** – prijave nastaju unutar FreeRTOS `portYIELD_FROM_ISR` makroa. Makro pripada dostavljenom MSVC portu i nije menjan.
- **Rule 11.2** – prijave nastaju zbog internih konverzija FreeRTOS opaque ručki za semafore, redove i tajmere. Aplikacija koristi javni FreeRTOS API na predviđen način.
- **Rule 21.1** – dostavljeni `HW_access.h` koristi zaštitni makro `_HW_ACCESS_H`, čiji naziv pripada rezervisanom prostoru identifikatora. Fajl simulatora nije mijenjan.
- **Directive 4.6** – preostale prijave potiču iz `HW_access.h`, FreeRTOS konfiguracionih makroa i tipova potrebnih za varijadne `printf` pozive.
- **Directive 4.8** – prijave se odnose na strukture iz Windows/MSVC zaglavlja.
- **Rule 8.9 (advisory)** – statički podaci stanja ostavljeni su na nivou modula jer predstavljaju trajno ili zajedničko stanje koje koriste FreeRTOS taskovi.

Odstupanja su dokumentovana i odnose se na komponente koje nisu dio aplikativnog koda i koje nisu menjane u okviru projekta.

---

## Rezultati

Projekat predstavlja funkcionalnu FreeRTOS aplikaciju za simulaciju merenja unutrašnje i spoljašnje temperature u automobilu.

Implementacija obuhvata senzorsku komunikaciju, filtriranje merenja pomoću pokretnog proseka, temperaturnu kalibraciju, LED indikaciju, alarm, 7-segmentni prikaz i PC komunikaciju.

Aplikativni kod je dodatno analiziran prema MISRA C:2012 pravilima, a preostala odstupanja su dokumentovana i obrazložena. 