# Omni-RC-Robot — firmware ESP32-S3

Firmware d'un **robot radiocommandé omnidirectionnel à 4 roues**, avec une interface Web
de diagnostic et de réglage hébergée directement sur la carte.

Deux protocoles de radiocommande sont supportés, **reconnus automatiquement au démarrage** :
**SBUS** (Futaba, FrSky) et **iBUS** (FlySky — FS-iA6B, FS-iA10B…).

---

## 1. Présentation

### Rôle du robot

Le robot se déplace dans toutes les directions grâce à quatre roues omnidirectionnelles :
translation avant/arrière, translation latérale, diagonales, rotation sur place, et
n'importe quelle combinaison translation + rotation.

Le pilotage se fait à la radiocommande. L'interface Web sert au **diagnostic, au réglage
et à la calibration** — jamais à conduire le robot.

### Architecture matérielle

| Élément | Modèle |
|---|---|
| Calculateur | Freenove ESP32-S3 WROOM (N8R8) |
| Radio | Récepteur RC en SBUS ou iBUS (testé : FlySky FS-iA6B) |
| Puissance | 4 × BTS7960 / IBT-2 |
| Motorisation | 4 moteurs DC + 4 roues omnidirectionnelles (géométrie ~60°) |

### Architecture logicielle

```
                          ┌──────────────┐
                          │   main.cpp   │  séquence de démarrage
                          └──────┬───────┘
                                 │
              ┌──────────────────┴───────────────────┐
              │                                      │
      cœur 1 — temps réel (200 Hz)          cœur 0 — réseau
              │                                      │
     ┌────────▼─────────┐                  ┌─────────▼────────┐
     │ RobotController  │                  │  WebInterface    │
     └────────┬─────────┘                  └─────────┬────────┘
              │                                      │
   ┌──────────┼──────────┬──────────────┐            │
   │          │          │              │            │
┌──▼────┐ ┌───▼─────┐ ┌──▼──────────┐ ┌─▼─────────┐  │
│ Sbus  │ │ Safety  │ │ OmniDrive   │ │  Motor    │  │
│Receiver│ │ Manager │ │ Mixer       │ │Controller │  │
└───────┘ └─────────┘ └─────────────┘ └───────────┘  │
                                                     │
              ┌──────────────────────────────────────┘
              │
     ┌────────▼─────────┐        ┌──────────┐
     │  ConfigManager   │        │  Logger  │
     │  (NVS + JSON)    │        │          │
     └──────────────────┘        └──────────┘
```

**La boucle de contrôle moteur tourne sur le cœur 1, le serveur Web sur le cœur 0.**
Une requête Web, même lente, ne peut donc jamais retarder le pilotage ni la détection
d'une perte radio.

---

## 2. Pré-requis

1. **VSCode** — <https://code.visualstudio.com/>
2. **Extension PlatformIO IDE** — à installer depuis le panneau Extensions de VSCode.
3. Brancher la carte. **Elle a deux ports USB-C**, et le choix détermine l'environnement à utiliser :

   | Port | Ce que Linux voit | Environnement PlatformIO |
   |---|---|---|
   | **UART** (pont CH343) | `/dev/ttyUSB0` | `freenove_esp32_s3_wroom` |
   | **USB** (natif, JTAG/CDC) | `/dev/ttyACM0` | `freenove_esp32_s3_wroom_usb` |

   Les deux permettent de téléverser. La différence est le terminal série : sur le port
   natif, `Serial` doit être redirigé vers le CDC USB, ce que fait l'environnement `_usb`.
   Avec le mauvais environnement, le téléversement marche mais **le terminal reste vide**.
4. Ouvrir le dossier `firmware/omni-rc-robot-frw/` dans VSCode.
   PlatformIO télécharge automatiquement la plateforme et les bibliothèques.
5. L'environnement à sélectionner est `freenove_esp32_s3_wroom` (c'est celui par défaut).

Sous Linux, si le port série n'est pas accessible :
`sudo usermod -a -G dialout $USER` puis se reconnecter.

---

## 3. Organisation du projet

```
platformio.ini            Configuration de build, dépendances, environnements.

include/pinout.h          RÉFÉRENCE UNIQUE DU CÂBLAGE. Tous les GPIO du projet.
include/robot_config.h    Structures de configuration et valeurs par défaut.
include/wifi_secrets.h    Identifiants Wi-Fi réels — NON SUIVI PAR GIT.
include/wifi_secrets.example.h  Modèle à copier pour créer le fichier ci-dessus.

src/main.cpp              Point d'entrée : setup(), loop(), création de la tâche temps réel.
src/robot_controller.cpp  Coordination générale : radio → sécurité → mixer → moteurs.
src/radio_receiver.cpp    Lecture et décodage de la radiocommande (SBUS et iBUS).
src/motor_controller.cpp  Pilotage des BTS7960 et rampe d'accélération.
src/omni_drive_mixer.cpp  Mathématiques du mixage omnidirectionnel (testable sur PC).
src/safety_manager.cpp    Machine d'états, armement, désarmement, failsafe.
src/config_manager.cpp    Persistance NVS, validation, import/export JSON.
src/web_interface.cpp     Wi-Fi, serveur HTTP, API et WebSocket.
src/logger.cpp            Journal : sortie série + tampon circulaire pour le Web.

data/index.html           Interface Web (une seule page, HTML/CSS/JS sans dépendance).
data/style.css
data/app.js

test/test_mixer/          Tests unitaires du mixer, exécutés sur PC.
```

---

## 4. Premier démarrage

> ### ⚠️ Sécurité — à lire avant le premier essai
>
> - **Roues levées.** Le robot doit reposer sur des cales, roues dans le vide.
> - **Puissance limitée.** Le firmware démarre à **50 %** (`maxMotorOutput`) ; le laisser bas.
> - **Robot désarmé** avant toute modification de configuration.
> - **Accès immédiat à l'alimentation** — interrupteur ou connecteur de batterie à portée de main.
> - Vérifier la réception SBUS **avant** de brancher la puissance des moteurs.

### Étapes

1. Ouvrir le dossier `firmware/omni-rc-robot-frw/` dans VSCode.
2. Laisser PlatformIO installer les dépendances (première ouverture uniquement).
3. Brancher l'ESP32-S3 sur le port USB **UART**.
4. **Compiler** : `pio run` (ou l'icône ✓ dans la barre PlatformIO).
5. **Téléverser le firmware** : `pio run -t upload`
   *(port USB natif : `pio run -e freenove_esp32_s3_wroom_usb -t upload`)*
6. **Téléverser l'interface Web** : `pio run -t uploadfs`
   *(dans VSCode : PlatformIO → Platform → **Upload Filesystem Image**)*
   Cette étape est **indispensable** : sans elle, le robot fonctionne mais affiche une
   page expliquant que l'interface est absente.
7. **Ouvrir le terminal série** : `pio device monitor` (115200 bauds).
8. **Vérifier le démarrage** — le récapitulatif doit s'afficher :

   ```
   [ 0.412][INFO][MAIN] === Omni-RC-Robot - firmware 0.1.0 ===
   [ 0.418][INFO][MAIN] SBUS sur GPIO 18, timeout 100 ms
   [ 0.425][INFO][MAIN] M0 : RPWM GPIO 4, LPWM GPIO 5, angle 210.0 deg, gain 1.00
   [ 1.204][INFO][WEB]  Connecte a 'monreseau_ext', adresse IP 192.168.1.42
   ```

9. **Se connecter au robot** (voir §5).
10. **Ouvrir l'interface** dans un navigateur.
11. **Vérifier la réception SBUS** dans le tableau de bord — allumer la radiocommande et
    contrôler que les voies bougent — **avant** d'activer les moteurs.

---

## 5. Réseau

Le robot dispose de deux modes, et le point d'accès reste **toujours disponible en secours**.

### Mode point d'accès (AP)

Le robot crée son propre réseau `RobotRC-XXXX` (suffixe dérivé de l'identifiant de puce).
Interface accessible sur **<http://192.168.4.1>**. Aucun accès Internet n'est nécessaire.

### Mode station (STA)

Le robot rejoint un réseau existant. L'interface est alors accessible sur l'IP affichée au
démarrage, ou sur **<http://omni-robot.local>** (mDNS).

**Si la connexion échoue dans le délai imparti, le robot recrée automatiquement son point
d'accès** — une mauvaise configuration réseau ne peut donc jamais rendre l'interface
inaccessible. En dernier recours, maintenir le bouton **BOOT** (GPIO 0) pendant 1 seconde
au démarrage restaure les valeurs par défaut.

### Identifiants Wi-Fi et Git

Les identifiants réels sont dans `include/wifi_secrets.h`, **exclu du dépôt par `.gitignore`**.
Sur une nouvelle machine :

```bash
cp include/wifi_secrets.example.h include/wifi_secrets.h
# puis renseigner le SSID et le mot de passe
```

Le projet compile même si ce fichier est absent : le robot démarre alors en point d'accès.

> **Attention à la casse du SSID.** Les noms de réseau y sont sensibles : `monreseau_ext` et
> `MonReseau_EXT` sont deux réseaux différents pour l'ESP32. En cas d'échec de connexion, le
> firmware balaye les réseaux visibles et l'indique explicitement dans le journal :
>
> ```
> [ERROR][WEB] Le reseau visible s'appelle 'MonReseau_EXT' et non 'monreseau_ext' :
>              seule la casse differe. Les SSID y sont sensibles.
> ```

Les mots de passe ne sortent **jamais en clair** de l'API Web : `/api/config` les remplace
par `********`. Renvoyer ce marqueur conserve le mot de passe déjà enregistré.

---

## 6. Modification des GPIO

**Tout le câblage est dans un seul fichier : [`include/pinout.h`](include/pinout.h).**
Aucun numéro de broche n'apparaît ailleurs dans le projet.

| Fonction | GPIO par défaut | À confirmer |
|---|---:|---|
| Entrée radio (SBUS/iBUS) | 18 | ✅ |
| M0 RPWM / LPWM | 1 / 2 | ✅ |
| M1 RPWM / LPWM | 6 / 7 | ✅ |
| M2 RPWM / LPWM | 15 / 16 | ✅ |
| M3 RPWM / LPWM | 17 / 8 | ✅ |
| M0..M3 R_EN / L_EN | −1 (câblés à +5 V) | ✅ |
| LED de statut | 48 (RGB embarquée) | — |
| Bouton reset config | 0 (BOOT) | — |

Les valeurs marquées ✅ sont des **valeurs de départ cohérentes avec la carte**, à confirmer
avec le câblage réel du robot.

**Broches à ne pas utiliser sur cette carte :** 19/20 (USB natif), 26–37 (flash et PSRAM),
43/44 (UART0 du terminal série), 0/45/46 (strapping), 38/39/40 (lecteur microSD), 48 (LED RGB).

> ### ⚠️ Conflit avec le connecteur caméra
>
> La caméra occupe les GPIO **4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18**.
>
> Toutes ne se valent pas :
>
> - **GPIO 4 (SIOD) et 5 (SIOC)** portent le bus **I2C** du capteur : résistances de tirage
>   soudées et puce toujours à l'écoute. Un signal PWM y est interprété comme du trafic I2C,
>   le capteur tire la ligne et la commande moteur est corrompue. **Inutilisables dès qu'une
>   caméra est branchée** — c'est pour cette raison que M0 a été déplacé sur GPIO 1 / 2.
> - Les autres sont des **sorties de données** que le capteur ne pilote pas tant qu'il n'a pas
>   été configuré par I2C. Elles restent utilisables : vérifié sur ce robot pour M1, M2, M3
>   et la radio.
>
> **Broches réellement libres de tout usage : 1, 2, 14, 21, 41, 42, 47.**

> **Deux contraintes matérielles importantes, vérifiées sur la carte :**
>
> 1. L'ESP32-S3 n'a que **8 canaux LEDC**. Les 4 moteurs × 2 broches PWM les consomment
>    **tous** — aucun autre périphérique ne peut utiliser `ledcSetup()`. C'est pourquoi la LED
>    de statut est pilotée en tout-ou-rien et non en PWM.
>
> 2. Le LEDC impose `fréquence × 2^résolution ≤ horloge source`. Sur cette puce l'horloge se
>    cale sur **40 MHz** (et non les 80 MHz du bus APB) : à 20 kHz, la résolution maximale est
>    donc de **10 bits**, pas 11. Une combinaison refusée ferait échouer les 8 canaux et
>    **aucun moteur ne répondrait**. `MotorController` éprouve la résolution sur le matériel au
>    démarrage et descend automatiquement jusqu'à une valeur acceptée, en le signalant.

Si les broches `R_EN` / `L_EN` sont câblées sur l'ESP32 plutôt que sur +5 V, remplacer les
`-1` correspondants par le numéro de GPIO : le firmware les pilotera automatiquement.

---

## 7. Calibration

### 7.1 Protocole du récepteur

Le firmware essaie SBUS puis iBUS au démarrage et garde celui qui produit des trames valides :

```
[INFO][RADIO] Aucune trame SBUS valide (1080 octets bruts lus au total)
[INFO][RADIO] UART1 sur GPIO 18 : iBUS, 115200 8N1 non inverse
[INFO][RADIO] Recepteur reconnu : iBUS
```

Les deux protocoles n'ont pas la même échelle de valeurs — SBUS 172…1811, iBUS 1000…2000 µs —
donc **changer de protocole remet la calibration des axes aux valeurs par défaut du nouveau**.
C'est signalé dans le journal ; il faut refaire la calibration après un changement de récepteur.

Le protocole peut aussi être imposé dans l'onglet *Radio*, en décochant la reconnaissance
automatique.

> **Le FlySky FS-iA6B ne sort pas de SBUS** : il fait du PPM et de l'iBUS. Il se branche
> directement sur GPIO 18, le firmware le reconnaît tout seul.

### 7.2 Calibration des axes

Onglet **Radio** de l'interface.

1. Le tableau de bord affiche les 16 voies brutes en direct — bouger un manche identifie
   immédiatement sa voie. Renseigner le numéro dans « Voie SBUS » de chaque axe.
2. Assistant automatique :
   - **1. Manches au centre** — relâcher tous les manches ;
   - **2. Balayer les extrêmes** — pousser chaque manche jusqu'aux butées ;
   - **3. Appliquer** — les min / centre / max sont reportés dans les champs.
3. Vérifier les valeurs, puis **Appliquer**, puis **Enregistrer**.

Affectation par défaut : CH1 = vx, CH2 = vy, CH4 = ω, CH5 = armement, CH6 = mode de vitesse.

Si **aucune trame n'est décodée**, le tableau de bord affiche les **octets bruts reçus**. C'est
le premier endroit à regarder :

| Ce que montre la trace | Cause probable |
|---|---|
| `0 octet` | rien n'est émis, mauvaise broche, ou récepteur non alimenté |
| octets présents, aucune trame valide | mauvais protocole, vitesse ou polarité |
| trames valides, cadence ~150 Hz | tout va bien |

L'outil `pio run -e radio_scan -t upload` va plus loin : il essaie toutes les combinaisons
vitesse / format / polarité courantes et affiche ce qu'il reçoit pour chacune. C'est ce qui a
permis d'identifier l'iBUS du FS-iA6B.

### 7.3 Sens des moteurs

Onglet **Moteurs**, test individuel (§8). Si une roue tourne à l'envers, cocher
**Inverser le sens** de la roue concernée dans l'onglet **Géométrie**.

### 7.4 Angles des roues et mixer

Onglet **Géométrie**.

Les presets **45°** (X-drive), **60°** (géométrie annoncée de ce robot) et **30°** pré-remplissent
les quatre angles. Chaque roue reste ensuite modifiable individuellement.

Le **simulateur de mixer** calcule les sorties théoriques **sans commander aucun moteur** :
c'est l'outil à utiliser pour valider mathématiquement le mixage avant tout essai réel.
Cliquer *Avant*, *Droite*, *Rotation gauche*… et vérifier que les signes correspondent au
comportement attendu.

### 7.5 Comportement de conduite

Onglet **Conduite** : puissance maximale, limites de translation et de rotation, mode lent,
rampes d'accélération et de décélération, paramètres de sécurité.

#### Seuil de démarrage moteur

Un moteur à courant continu avec réducteur ne démarre pas en dessous d'un certain rapport
cyclique : il chauffe et bourdonne sans tourner. Le robot semble alors « mort » à faible
commande, alors que l'électronique fonctionne.

`minMotorOutput` compense ce seuil. La plage utile est comprimée de `[0, maxMotorOutput]`
vers `[minMotorOutput, maxMotorOutput]` :

```
sortie = minMotorOutput + (maxMotorOutput − minMotorOutput) × (commande / maxMotorOutput)
```

Toute commande non nulle produit donc au moins `minMotorOutput`, **sans jamais dépasser**
`maxMotorOutput`. Une commande nulle reste nulle.

**Comment trouver la valeur :** onglet *Moteurs*, baissez la puissance du test moteur
(`motorTestMaxOutput`) jusqu'à ce que la roue cesse de tourner. La valeur juste au-dessus est
votre seuil. Typiquement **0,15 à 0,25** sur des moteurs 36GP-555.

Mettre `0` désactive la compensation.

> **Attention aux réductions qui s'accumulent.** La sortie réelle d'une roue est le produit
> de plusieurs facteurs : `manche × gain d'axe × limite translation × mode lent × projection
> géométrique × gain de roue × puissance maximale`. Avec le mode lent à 0,40 et la puissance
> à 0,50, un manche à fond ne donne que 0,10 sur une roue à 60° — en dessous du seuil de
> démarrage de la plupart des moteurs. Le tableau de bord affiche la sortie du mixer en
> permanence, **même robot désarmé** : c'est là qu'on voit ce que chaque roue recevrait.

---

## 8. Procédure de test des quatre moteurs

> **Roues levées et robot calé.** Ce test sert uniquement à vérifier le câblage et le sens.

1. Robot **DISARMED** (interrupteur d'armement de la radio au repos).
2. Onglet **Moteurs**.
3. Cocher **« Je confirme que les roues sont levées et le robot désarmé »**.
4. **Maintenir** le bouton `+` ou `−` d'un moteur : il tourne à puissance limitée (20 % par défaut).
5. Vérifier pour chaque moteur M0 → M3 : **le bon moteur tourne**, et **dans le bon sens**.
6. Relâcher : le moteur s'arrête immédiatement.

Le mode test s'arrête automatiquement si :

- le bouton est relâché ou la page perd le focus ;
- la connexion Web est perdue ;
- le délai maximal est écoulé (3 s par défaut) ;
- l'armement est demandé à la radiocommande.

Le test est **refusé** si le robot n'est pas DISARMED ou si l'interrupteur d'armement est actif.

---

## 9. Procédure de test du failsafe

À réaliser **roues levées**, avant tout essai au sol.

| Étape | Action | Résultat attendu |
|---|---|---|
| 1 | Armer le robot, appliquer une petite commande | Les roues tournent, état `ARMED` |
| 2 | **Éteindre la radiocommande** | Moteurs arrêtés **immédiatement**, état `FAILSAFE`, LED rouge rapide |
| 3 | Rallumer la radiocommande | État `DISARMED` — **pas** de redémarrage automatique des moteurs |
| 4 | Débrancher le fil de données du récepteur | Même comportement qu'à l'étape 2 |
| 5 | Appuyer sur RESET de l'ESP32 | Aucune activation moteur au démarrage |

Le journal doit afficher :

```
[ERROR][SAFETY] ARMED -> FAILSAFE : aucune trame radio valide dans le delai imparti, moteurs coupes
```

Le retour de `FAILSAFE` vers `ARMED` est **impossible directement** : la radio doit être
retrouvée **et** l'interrupteur d'armement repassé au repos.

---

## 10. Description mathématique du mixer

### Convention de coordonnées (robot vu de dessus)

```
                     +Y (avant)
                      ↑
          M0          |          M1
                      |
     -X ──────────────O──────────────→ +X (droite)
                      |
          M3          |          M2
                      ↓
                     -Y (arrière)
```

- `vx > 0` → déplacement vers la **droite**
- `vy > 0` → déplacement vers l'**avant**
- `ω > 0` → rotation **antihoraire**

Les angles sont mesurés **en degrés, dans le sens antihoraire, depuis l'axe +X**.
0° pointe à droite, 90° vers l'avant.

### Formule

Pour chaque roue *i*, `driveAngleDeg` est l'angle θᵢ de sa **direction motrice** : la direction
dans laquelle son point de contact pousse le châssis quand le moteur tourne positivement.

La commande de la roue est la projection du vecteur vitesse demandé sur cette direction :

```
mᵢ = vx · cos(θᵢ) + vy · sin(θᵢ) + ω · rotationGainᵢ
```

### Normalisation

```
maxAbs = max(|m₀|, |m₁|, |m₂|, |m₃|)
si maxAbs > 1 :  mᵢ ← mᵢ / maxAbs
```

La direction globale demandée est ainsi **préservée**, au prix d'une réduction homogène de la
vitesse, plutôt que de saturer une seule roue et de dévier de la trajectoire.

Puis, pour chaque roue : gain individuel, inversion éventuelle, et plafond `maxMotorOutput`.

### Angles par défaut

`applySymmetricWheelGeometry()` répartit les angles symétriquement à partir d'un seul
paramètre — l'angle **a** entre la direction motrice et l'axe longitudinal :

| Roue | Angle | a = 45° (X-drive) | a = 60° (défaut) |
|---|---|---:|---:|
| M0 avant gauche | 270° − a | 225° | 210° |
| M1 avant droit | 90° + a | 135° | 150° |
| M2 arrière droit | 90° − a | 45° | 30° |
| M3 arrière gauche | 270° + a | 315° | 330° |

> **Ce mixage est volontairement paramétrique.** Aucune formule figée de Mecanum ou de X-drive
> n'est supposée : la signification physique exacte du « 60° » doit être confirmée sur le robot
> réel, et l'adaptation ne demande que d'ajuster les angles depuis l'interface Web.

### Vérification mathématique

Les tests unitaires tournent **sur PC, sans carte** :

```bash
pio test -e native_mixer_test
```

Ils vérifient notamment qu'une translation latérale ne produit **aucune** composante
longitudinale, qu'une rotation ne produit **aucune** translation, et qu'aucune combinaison
de commandes ne sort de la plage −1…+1.

---

## 11. Machine d'états

```
   BOOT
     │ initialisation terminée
     ▼
WAITING_FOR_RADIO
     │ liaison SBUS établie
     ▼
  DISARMED ◄──────────────────────┐
     │  ▲                         │
     │  │ interrupteur relâché    │ radio retrouvée
     │  │                         │ ET armement au repos
     │  │                         │
     ▼  │                         │
   ARMED ──── perte radio ───► FAILSAFE
     ▲                             ▲
     │                             │
  DISARMED ─► MOTOR_TEST ──────────┘
```

**Conditions d'armement** (toutes requises, maintenues 500 ms) :
liaison radio valide · aucun failsafe · manches au neutre · interrupteur d'armement actif.

> **Note iBUS.** Le protocole iBUS ne transporte aucun indicateur de failsafe, contrairement au
> SBUS. La perte de liaison est alors détectée uniquement par le **chien de garde temporel**
> (`frameTimeoutMs`, 100 ms par défaut) — ce qui reste suffisant, mais rend le test du failsafe
> (§9) d'autant plus important à valider.

**Priorité impérative des commandes :**

```
FAILSAFE  >  DÉSARMEMENT  >  TEST MOTEUR  >  COMMANDE SBUS
```

Aucune fonction de l'interface Web ne peut contourner un failsafe.

### LED de statut

| Motif | État |
|---|---|
| Double flash jaune | attente de la radio |
| Clignotement lent bleu | DISARMED |
| Vert fixe | ARMED |
| Clignotement rapide rouge | FAILSAFE |
| Clignotement orange | test moteur |

---

## 12. API HTTP

| Méthode | Route | Rôle |
|---|---|---|
| `GET` | `/api/status` | Télémétrie complète |
| `GET` | `/api/system` | Diagnostic système |
| `GET` | `/api/config` | Configuration (mots de passe masqués) |
| `POST` | `/api/config` | Modifie la configuration en RAM |
| `POST` | `/api/config/save` | Écrit la configuration en NVS |
| `POST` | `/api/config/reset` | Restaure les valeurs par défaut |
| `GET` | `/api/logs?since=N` | Journal depuis le numéro de séquence N |
| `POST` | `/api/motor-test` | Test moteur `{"motor":0,"output":0.2}` |
| `POST` | `/api/motor-test/stop` | Arrête le test moteur |
| `POST` | `/api/disarm` | Désarmement immédiat |
| `POST` | `/api/mixer-simulate` | Simule le mixer, **sans commander les moteurs** |
| `POST` | `/api/reboot` | Redémarre la carte |
| `WS` | `/ws` | Télémétrie (10 Hz) et journal en temps réel |

Les modifications de configuration restent **en RAM** jusqu'à `/api/config/save` : déplacer un
curseur n'use pas la mémoire flash.

---

## 13. Commandes utiles

```bash
pio run                        # compiler le firmware
pio run -t upload              # téléverser le firmware
pio run -t uploadfs            # téléverser l'interface Web (data/)
pio device monitor             # terminal série (115200 bauds)
pio device monitor -e freenove_esp32_s3_wroom_usb   # si branché sur le port USB natif
pio test -e native_mixer_test  # tests du mixer sur PC
pio run -e radio_scan -t upload  # identifier le protocole d'un récepteur inconnu
pio run -t clean               # nettoyer
```

---

## 14. Évolutions prévues par l'architecture

Non implémentées, mais l'architecture reste compatible : encodeurs et odométrie, IMU et
correction de cap, PID de vitesse, mesure de tension batterie et arrêt sur batterie faible,
capteurs de distance, écran OLED, buzzer, mise à jour OTA, ESP-NOW, pilotage semi-autonome.
