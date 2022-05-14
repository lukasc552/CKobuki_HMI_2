# Dokumentácia zadanie 2. HMI

## Opis úlohy:

Zadaním bolo navrhnúť používateľské rozhranie pre semi-autonómne riadenie mobilného robota typu Kobuki. Robot má za úlohu mapovať prostredie a zároveň byť schopný sa autonómne dostať na určenú pozíciu. Keďže medzi robotom a operátorom je dopravné oneskorenie, operátor len zadá príkaz kam sa má robot presunúť a celé riadenie vykoná robot sám.

## Používateľská dokumentácia:

Pre správne fungovanie programu je potrebné vopred spustiť:

- Kobuki simulátor
- Následne spustiť samotný program

Používateľské rozhranie sa skladá z:

- 75% mapa spolu s dátami z lidaru
- kamera o rozlíšení 4:3
- STOP tlačidlo pre núdzové zastavenie
- Priestor pre výpis aktuálnych informácií pre operátora

Môžeme to vidieť na nasledujúcom obrázku:

![Optional Text](Img/ui2_uloha_splnena.jpg)

Na tomto obrázku taktiž vidíme, že robot aktuálne splnil úlohu a to, že dosiahol bod, ktorý mu zadal operátor kliknutím kdekoľvek do čierneho framu. Taktiež sú v tomto frame zvizualizované miesta, cez ktoré už robot prešiel a vyhlásil za splnené.

Úloha mapovanie sa vykonáva neustále počas behu programu, keď prídu nové dáta z lidaru sa mapa aktualizuje. 
