# Librairies KiCad maison — Omni-RC-Robot

Librairies partagées par tous les projets sous `ecad/`.
Elles sont déclarées **au niveau projet** dans `ecad/omni-board/sym-lib-table`
et `ecad/omni-board/fp-lib-table` via `${KIPRJMOD}/../lib/...`
(rien à installer dans les préférences globales de KiCad).

| Nick             | Contenu                          |
|------------------|----------------------------------|
| `omni-symbols`   | `omni-symbols.kicad_sym`         |
| `omni-footprints`| `omni-footprints.pretty/`        |

## ESP32-C3-Zero (Waveshare)

Module ESP32-C3FH4, 18 broches, USB-C, WS2812 sur GP10.
Datasheet / wiki : https://www.waveshare.com/wiki/ESP32-C3-Zero

### Brochage

| Pin | Nom  | | Pin | Nom  |
|-----|------|-|-----|------|
| 1   | 5V   | | 18  | GP21 / UART0 TX |
| 2   | GND  | | 17  | GP20 / UART0 RX |
| 3   | 3V3 (out) | | 16 | GP19 / USB D+ |
| 4   | GP0  | | 15  | GP18 / USB D- |
| 5   | GP1  | | 14  | GP10 (WS2812 DIN) |
| 6   | GP2  | | 13  | GP9  (BOOT) |
| 7   | GP3  | | 12  | GP8  |
| 8   | GP4  | | 11  | GP7  |
| 9   | GP5  | | 10  | GP6  |

Pin 1 en haut à gauche, USB-C en haut ; numérotation dans le sens antihoraire
(1→9 colonne gauche de haut en bas, 10→18 colonne droite de bas en haut).

### Points d'attention design

- **Alim** : pin 1 = **5V d'entrée** (LDO à bord), pin 3 = **3V3 en sortie**.
  Ne pas alimenter par les deux en même temps.
- **Strapping pins** (état lu au reset, à ne pas tirer en force) : GP2, GP8, GP9.
  GP9 = BOOT (bouton présent sur le module) → doit être haut au démarrage.
- **GP18 / GP19** sont les lignes USB natives : les laisser libres si on veut
  garder l'USB-C du module.
- **GP10** pilote la LED WS2812 embarquée en plus d'être sorti sur pin 14.
- **ADC** : GP0…GP4 seulement.
- 4 sorties PWM par driver BTS7960 (RPWM/LPWM/R_EN/L_EN) × 4 moteurs = 16 signaux :
  il n'y a que 13 GPIO réellement libres si l'USB est conservé. Prévoir de câbler
  R_EN/L_EN ensemble (1 signal EN par driver) → 3 signaux/moteur = 12 GPIO.

---

## ESP32-S3-Zero (Waveshare)

Module ESP32-S3FH4R2 (4 Mo flash + 2 Mo PSRAM), USB-C natif, WS2812 sur GP21.
Wiki : https://www.waveshare.com/wiki/ESP32-S3-Zero

**Mécaniquement identique au C3-Zero** (même plan de cotes), mais **pas
compatible broche à broche** : GP0 n'est pas sorti, la numérotation des GPIO
diffère, et UART0 est en haut à droite.

### Brochage (18 broches de bord)

| Pin | Nom  | | Pin | Nom  |
|-----|------|-|-----|------|
| 1   | 5V   | | 18  | GP43 / UART0 TX |
| 2   | GND  | | 17  | GP44 / UART0 RX |
| 3   | 3V3 (out) | | 16 | GP13 |
| 4   | GP1  | | 15  | GP12 |
| 5   | GP2  | | 14  | GP11 |
| 6   | GP3  | | 13  | GP10 |
| 7   | GP4  | | 12  | GP9  |
| 8   | GP5  | | 11  | GP8  |
| 9   | GP6  | | 10  | GP7  |

GP1…GP13 ont tous une entrée ADC (ADC1 pour GP1-10, ADC2 pour GP11-13).

### ⚠️ Pastilles non modélisées

Le module expose en plus, **hors des deux rangées de bord** :
- bord bas : **GP14, GP15, GP16**
- face arrière : **GP17, GP18, GP38, GP39, GP40, GP41, GP42, GP45**

Elles **ne sont pas** dans l'empreinte : le plan de cotes Waveshare dont on
dispose ne donne pas leur position. À ajouter si besoin, cotes à relever.
Le symbole s'arrête donc lui aussi à 18 broches, pour rester cohérent avec
l'empreinte (sinon KiCad remonte une erreur « pin not found in footprint »).

### Points d'attention design

- **Strapping pins ESP32-S3** : GP0 (BOOT, non sorti), **GP3** (sélection source
  JTAG, sans pull interne — « don't care » si laissé flottant), **GP45**
  (tension VDD_SPI, doit rester **bas** au reset — sur pastille arrière, à
  éviter), GP46 (non sorti). Le lot GP1/GP2/GP4…GP13 est **sans contrainte de
  boot** — c'est bien plus confortable que sur le C3.
- **USB natif sur GP19/GP20, non sortis** → contrairement au C3-Zero, garder
  l'USB-C ne coûte aucun GPIO.
- **PWM** : l'S3 a **2 unités MCPWM (12 sorties)** + **8 canaux LEDC**. De quoi
  piloter 4 × BTS7960 en 8 PWM matériels sans multiplexeur externe, ce que le
  C3 ne peut pas faire (6 canaux LEDC, pas de MCPWM).
- **Alim** : pin 1 = 5V d'entrée, pin 3 = 3V3 en sortie. Jamais les deux.

### Empreintes (communes aux deux modules)

| Empreinte | Usage |
|-----------|-------|
| `ESP32-C3-Zero_THT` / `ESP32-S3-Zero_THT` (défaut) | trous métallisés Ø1.0 mm, barrettes 2×9 au pas 2.54 ou soudure directe |
| `ESP32-C3-Zero_Castellated` / `ESP32-S3-Zero_Castellated` | pastilles CMS 3.0 × 1.6 mm à cheval sur le bord, refusion à plat |

Géométrie (relevé du plan Waveshare, valable pour les deux modules) :
carte **18.0 × 23.5 mm**, pas **2.54 mm**, entraxe des rangées **15.24 mm**
(= 18.0 − 2 × 1.38), pin 1 à **1.59 mm** du bord haut, congés **R1.00**.
Le connecteur USB-C déborde du bord haut → le courtyard le couvre, garder la
zone dégagée sur la carte porteuse.
