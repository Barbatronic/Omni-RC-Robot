# Spécification firmware — Robot RC omnidirectionnel 4 roues
## ESP32-S3 Freenove WROOM · SBUS · BTS7960 · Interface Web embarquée

> Document destiné à un agent de développement dans VSCode.
>
> Objectif : développer un firmware robuste, configurable et documenté pour un robot radiocommandé omnidirectionnel à 4 roues.

---

# 1. Objectif général

Développer le firmware complet d'un robot RC omnidirectionnel équipé de :

- **1 carte Freenove ESP32-S3 WROOM** ;
- **1 récepteur RC utilisant le protocole SBUS** ;
- **4 moteurs DC** ;
- **4 drivers BTS7960 / IBT-2**, un par moteur ;
- **4 roues omnidirectionnelles**, disposées selon une géométrie à environ **60°** qui devra rester configurable dans le logiciel ;
- une **interface Web hébergée directement sur l'ESP32-S3**.

Le robot doit pouvoir réaliser :

- translation avant / arrière ;
- translation gauche / droite ;
- mouvements diagonaux ;
- rotation sur lui-même ;
- combinaison simultanée translation + rotation.

Le firmware doit être pensé dès le départ comme un projet maintenable, paramétrable et diagnostiquable.

---

# 2. Environnement de développement

L'environnement de développement imposé est :

- **VSCode**
- **PlatformIO**
- framework **Arduino ESP32**
- cible **ESP32-S3**

Le projet doit être directement ouvrable, compilable et téléversable depuis PlatformIO dans VSCode.

Le fichier `platformio.ini` doit être fourni, fonctionnel et commenté lorsque certains paramètres ne sont pas évidents.

L'utilisation de l'IDE Arduino n'est pas l'environnement cible du projet.

Le projet doit compiler sans dépendre d'un IDE propriétaire.

Une structure de projet claire est requise.

Exemple :

```text
/
├── platformio.ini
├── README.md
├── src/
│   ├── main.cpp
│   ├── robot_controller.cpp
│   ├── sbus_receiver.cpp
│   ├── motor_controller.cpp
│   ├── omni_drive_mixer.cpp
│   ├── safety_manager.cpp
│   ├── config_manager.cpp
│   ├── web_interface.cpp
│   └── logger.cpp
├── include/
│   ├── robot_controller.h
│   ├── sbus_receiver.h
│   ├── motor_controller.h
│   ├── omni_drive_mixer.h
│   ├── safety_manager.h
│   ├── config_manager.h
│   ├── web_interface.h
│   ├── logger.h
│   ├── pinout.h
│   └── robot_config.h
└── data/
    ├── index.html
    ├── app.js
    └── style.css
```

La structure exacte peut évoluer, mais il faut éviter un unique `main.cpp` monolithique.

---

# 2.1 Lisibilité, qualité et niveau technique attendu

Le projet doit rester **compréhensible par une personne ayant des bases en C/C++**, sans pour autant être simplifié artificiellement.

L'objectif n'est pas de produire du code "débutant" ou scolaire, mais un code :

- propre ;
- structuré ;
- robuste ;
- maintenable ;
- suffisamment moderne ;
- facile à parcourir et à comprendre.

L'agent peut utiliser les mécanismes C++ pertinents lorsqu'ils apportent un réel intérêt au projet :

- classes ;
- structures ;
- `enum class` ;
- références ;
- `constexpr` ;
- encapsulation ;
- séparation interface / implémentation ;
- objets représentant les sous-systèmes du robot.

En revanche, éviter la complexité gratuite :

- templates génériques sans nécessité ;
- hiérarchies de classes profondes ;
- abstraction excessive ;
- multiplication de couches logicielles ;
- patterns complexes uniquement pour "faire propre" ;
- architecture conçue comme une grosse application serveur alors qu'il s'agit d'un firmware embarqué.

Le principe à suivre est :

> **Utiliser le niveau de structuration nécessaire pour obtenir un firmware propre et robuste, mais pas davantage.**

---

# 2.1.1 Nommage des variables et fonctions

Le code reste entièrement nommé en **anglais**.

Les noms doivent être explicites et cohérents dans tout le projet.

Éviter les noms trop courts ou ambigus :

```cpp
v
val
tmp
data
proc
upd
ctrl
cfg
```

sauf lorsque le contexte les rend réellement évidents, par exemple un indice de boucle local.

Privilégier :

```cpp
rawSbusValue
normalizedThrottle
translationX
translationY
rotationCommand
motorOutput
radioTimeoutMs
isRobotArmed
lastValidSbusFrameTime
```

Pour les fonctions, utiliser un verbe décrivant clairement l'action :

```cpp
readSbusFrame()
normalizeChannelValue()
computeWheelCommands()
applyMotorOutputs()
updateFailsafeState()
loadRobotConfiguration()
saveRobotConfiguration()
```

Les fonctions de test booléen doivent être lisibles naturellement :

```cpp
isSbusConnected()
isRobotArmed()
hasRadioTimedOut()
canEnterMotorTestMode()
```

Éviter :

```cpp
check()
process()
doStuff()
run()
handle()
```

lorsque le nom ne permet pas de comprendre ce qui est réellement traité.

---

# 2.1.2 Conventions de nommage

Adopter une convention unique et la conserver partout.

Recommandation :

```text
Classes / structures / enums :
PascalCase

Fonctions :
camelCase

Variables :
camelCase

Constantes constexpr :
camelCase ou UPPER_SNAKE_CASE,
mais choisir une seule convention pour tout le projet

Fichiers :
snake_case
```

Exemple :

```cpp
class SbusReceiver;
class MotorController;
class OmniDriveMixer;

struct RobotConfiguration;
struct WheelConfiguration;

enum class RobotState;

float translationX;
float rotationCommand;

void updateRobotControl();
```

Ne pas mélanger plusieurs conventions sans raison.

---

# 2.1.3 Taille et responsabilité des fonctions

Une fonction doit avoir une responsabilité identifiable.

Éviter les fonctions de plusieurs centaines de lignes qui gèrent simultanément :

- SBUS ;
- sécurité ;
- calcul cinématique ;
- moteurs ;
- interface Web ;
- logs.

À l'inverse, ne pas découper chaque opération triviale dans une fonction distincte uniquement pour réduire artificiellement la taille des fonctions.

Le découpage doit suivre la logique fonctionnelle du robot.

---

# 2.1.4 Niveau d'abstraction

Le code doit permettre de lire le fonctionnement général sans devoir suivre dix niveaux d'appels.

Par exemple :

```cpp
void updateRobotControl()
{
    updateRadioInput();
    updateSafetyState();

    if (!isRobotArmed()) {
        stopAllMotors();
        return;
    }

    updateDriveCommands();
    applyMotorOutputs();
}
```

est préférable à une chaîne d'abstractions où le comportement réel devient difficile à retrouver.

---

# 2.2 Documentation pédagogique dans le code

Chaque module doit commencer par un commentaire décrivant :

- son rôle ;
- ses entrées ;
- ses sorties ;
- les autres modules avec lesquels il communique ;
- les éventuelles contraintes matérielles importantes.

Exemple :

```cpp
/**
 * @file sbus_receiver.cpp
 *
 * @brief Gestion de la réception radio SBUS.
 *
 * Ce module lit les trames reçues sur l'UART dédiée au récepteur RC,
 * extrait les voies SBUS et fournit au reste du programme des valeurs
 * normalisées entre -1.0 et +1.0.
 *
 * Il est également responsable de la détection :
 * - des pertes de trames ;
 * - du failsafe SBUS ;
 * - du timeout radio.
 */
```

Les fonctions publiques importantes doivent être documentées.

Exemple :

```cpp
/**
 * @brief Convertit une voie SBUS brute en valeur normalisée.
 *
 * @param rawValue Valeur brute reçue du récepteur.
 * @param calibration Calibration min / centre / max de la voie.
 * @return Valeur comprise entre -1.0 et +1.0.
 */
float normalizeSbusChannel(
    uint16_t rawValue,
    const ChannelCalibration& calibration
);
```

Il n'est pas nécessaire d'ajouter des commentaires sur des instructions triviales.

---

# 2.3 README pédagogique

Le `README.md` du projet doit permettre à un nouvel utilisateur de comprendre le projet sans avoir besoin de lire immédiatement tout le code.

Il doit contenir au minimum :

## Présentation

- rôle du robot ;
- architecture matérielle ;
- architecture logicielle générale.

## Pré-requis

- installation VSCode ;
- installation PlatformIO ;
- branchement USB ;
- sélection du bon environnement PlatformIO.

## Organisation du projet

Expliquer simplement le rôle de chaque fichier/dossier.

Exemple :

```text
src/main.cpp
    Point d'entrée du programme.

src/sbus.cpp
    Lecture et décodage de la radiocommande.

src/motor.cpp
    Pilotage d'un moteur via BTS7960.

src/mixer.cpp
    Transformation vx / vy / omega en commandes des quatre moteurs.

src/web_server.cpp
    Interface Web du robot.

src/config.cpp
    Chargement et sauvegarde de la configuration.
```

## Premier démarrage

Décrire étape par étape :

1. ouvrir le projet dans VSCode ;
2. laisser PlatformIO installer les dépendances ;
3. brancher l'ESP32-S3 ;
4. compiler ;
5. téléverser ;
6. ouvrir le terminal série ;
7. vérifier le démarrage ;
8. se connecter au Wi-Fi du robot ;
9. ouvrir la page Web ;
10. vérifier la réception SBUS avant de brancher ou d'activer les moteurs.

## Modification des GPIO

Expliquer précisément où modifier les broches.

## Calibration

Expliquer :

- calibration SBUS ;
- inversion moteurs ;
- réglage des angles de roues ;
- réglage du mixer.

## Sécurité

Ajouter un encadré clair indiquant que les premiers essais doivent être réalisés :

- roues levées ;
- puissance moteur limitée ;
- robot désarmé avant toute modification ;
- avec accès immédiat à l'alimentation.

---


# 2.3.1 Fichier de pinout séparé

Le projet doit obligatoirement contenir un **fichier de pinout séparé**.

Objectif :

- retrouver immédiatement toutes les broches utilisées ;
- modifier le câblage sans parcourir plusieurs fichiers ;
- éviter les GPIO dispersés dans les modules métiers.

Recommandation :

```text
include/pinout.h
```

Ce fichier doit regrouper au minimum :

- la broche RX SBUS ;
- les broches RPWM / LPWM / éventuellement REN / LEN de chaque moteur ;
- la LED de statut ;
- les éventuelles broches de boutons ou entrées système.

Exemple de contenu attendu :

```cpp
#pragma once

struct MotorPins
{
    int rightPwmPin;
    int leftPwmPin;
    int rightEnablePin;
    int leftEnablePin;
};

namespace Pinout
{
    constexpr int sbusRxPin = -1;
    constexpr int statusLedPin = -1;

    constexpr MotorPins motor0Pins = { -1, -1, -1, -1 };
    constexpr MotorPins motor1Pins = { -1, -1, -1, -1 };
    constexpr MotorPins motor2Pins = { -1, -1, -1, -1 };
    constexpr MotorPins motor3Pins = { -1, -1, -1, -1 };
}
```

Le format exact peut varier, mais le principe est imposé :

> **tout le pinout matériel doit être centralisé dans un fichier dédié**.

---

# 2.3.2 Fichier de configuration séparé

Le projet doit également contenir un **fichier de configuration séparé** des modules fonctionnels.

Objectif :

- centraliser les structures de configuration ;
- centraliser les valeurs par défaut ;
- distinguer clairement :
  - le câblage matériel ;
  - les paramètres utilisateurs ;
  - la logique du firmware.

Recommandation :

```text
include/robot_config.h
src/config_manager.cpp
```

Le fichier de configuration doit regrouper au minimum :

- les structures `RobotConfiguration`, `RadioConfiguration`, `MotorConfiguration`, `WheelConfiguration`, etc. ;
- les valeurs par défaut ;
- les bornes minimales / maximales utiles ;
- les versions de configuration.

Exemple d'organisation :

```cpp
struct RadioConfiguration
{
    int translationXChannel;
    int translationYChannel;
    int rotationChannel;
    int armChannel;
    float deadband;
    float translationExpo;
    float rotationExpo;
};

struct WheelConfiguration
{
    float driveAngleDeg;
    float rotationGain;
    float outputGain;
    bool inverted;
};

struct RobotConfiguration
{
    uint32_t version;
    RadioConfiguration radio;
    WheelConfiguration wheels[4];
    float maxMotorOutput;
    uint32_t sbusTimeoutMs;
};
```

Le projet peut ensuite utiliser `ConfigManager` pour :

- charger cette configuration depuis la NVS ;
- la modifier ;
- la sauvegarder ;
- la restaurer par défaut.

Important :

> Le fichier de configuration ne doit pas être confondu avec le pinout.
>
> - `pinout.h` = brochage matériel
> - `robot_config.h` = paramètres et structures de configuration

---

# 2.4 Simplicité de `main.cpp`

Le fichier `main.cpp` doit rester volontairement très simple.

Il doit principalement montrer la séquence générale du firmware.

Exemple conceptuel :

```cpp
void setup()
{
    initializeSystem();
    loadConfiguration();
    initializeRadio();
    initializeMotors();
    initializeWebInterface();
}

void loop()
{
    updateRadio();
    updateSafety();
    updateRobotControl();
    updateWebInterface();
}
```

La logique détaillée ne doit pas être dispersée directement dans `loop()`.

Un lecteur débutant doit pouvoir comprendre le fonctionnement général du robot simplement en lisant `setup()` et `loop()`.

---

# 2.5 Utilisation de bibliothèques

Limiter le nombre de bibliothèques externes.

Avant d'ajouter une dépendance, vérifier si :

- elle apporte réellement une simplification ;
- elle est maintenue ;
- elle fonctionne correctement sur ESP32-S3 ;
- elle est disponible facilement via PlatformIO.

Toutes les dépendances doivent être déclarées dans :

```ini
lib_deps =
```

du fichier `platformio.ini`.

Éviter les installations manuelles de bibliothèques dans le dossier Arduino de l'utilisateur.

Le projet doit pouvoir être cloné sur un autre ordinateur puis compilé directement par PlatformIO.

---

# 2.6 Gestion des erreurs compréhensible

Les erreurs doivent produire des messages explicites.

À éviter :

```text
ERR 14
```

À privilégier :

```text
[ERROR][SBUS] Aucun signal radio reçu depuis 150 ms
[ERROR][SAFETY] Passage en FAILSAFE : moteurs arrêtés
```

Lorsqu'une erreur empêche le fonctionnement du robot, expliquer :

- ce qui a été détecté ;
- la conséquence ;
- si possible, ce que l'utilisateur doit vérifier.

---


# 3. Architecture logicielle souhaitée

Le firmware doit être séparé en plusieurs modules afin de faciliter :

- la lecture ;
- la maintenance ;
- les tests ;
- l'évolution du robot.

Cependant, il faut éviter une architecture trop tentaculaire.

Le but n'est pas de créer un fichier par petite fonction ou un grand nombre de classes minuscules.

Un module doit correspondre à un **sous-système réel ou à une responsabilité importante du robot**.

Une organisation de l'ordre de **6 à 10 modules principaux** est préférable à plusieurs dizaines de fichiers.

---

## 3.1 Organisation recommandée

Une architecture de départ possible :

```text
src/
├── main.cpp
├── robot_controller.cpp
├── sbus_receiver.cpp
├── motor_controller.cpp
├── omni_drive_mixer.cpp
├── safety_manager.cpp
├── config_manager.cpp
├── web_interface.cpp
└── logger.cpp

include/
├── robot_controller.h
├── sbus_receiver.h
├── motor_controller.h
├── omni_drive_mixer.h
├── safety_manager.h
├── config_manager.h
├── web_interface.h
├── logger.h
├── pinout.h
└── robot_config.h
```

Cette organisation est indicative.

L'agent peut fusionner certains modules si cela améliore réellement la compréhension.

Par exemple :

- la gestion de la sécurité peut être intégrée au `RobotController` si elle reste simple ;
- le logger peut rester très léger et ne pas nécessiter une classe complexe ;
- la télémétrie Web peut rester dans `WebInterface`.

À l'inverse, un fichier devenu trop volumineux ou gérant plusieurs responsabilités distinctes pourra être séparé.

---

## 3.2 `main.cpp`

`main.cpp` doit rester le point d'entrée lisible du firmware.

Il doit principalement :

- initialiser le système ;
- appeler les grandes fonctions de mise à jour ;
- rendre visible le fonctionnement général.

Il ne doit pas contenir toute l'implémentation du robot.

Exemple :

```cpp
void setup()
{
    robotController.begin();
}

void loop()
{
    robotController.update();
}
```

ou une version légèrement plus détaillée si cela améliore la compréhension.

Le but est qu'un lecteur puisse identifier très rapidement où commence le programme et où chercher ensuite.

---

## 3.3 `RobotController`

Ce module assure la coordination générale du robot.

Responsabilités possibles :

- machine d'états ;
- orchestration de la réception radio ;
- sécurité ;
- préparation des commandes ;
- appel du mixer ;
- transmission des consignes aux moteurs.

Il ne doit pas réimplémenter les détails internes des autres modules.

---

## 3.4 `SbusReceiver`

Responsabilités :

- initialisation de l'UART ;
- réception des trames SBUS ;
- décodage des voies ;
- récupération des indicateurs :
  - frame lost ;
  - failsafe ;
- stockage de l'heure de la dernière trame valide ;
- accès aux valeurs brutes.

La conversion des voies en commandes utilisateur peut rester ici ou être réalisée par `RobotController`, selon ce qui produit le code le plus lisible.

---

## 3.5 `MotorController`

Représenter proprement les quatre moteurs et leurs BTS7960.

Il est pertinent d'avoir une classe représentant un moteur, par exemple :

```cpp
class Motor
{
public:
    void begin();
    void setOutput(float output);
    void stop();

private:
    MotorPins pins;
    bool inverted;
};
```

Puis un gestionnaire ou simplement un tableau de quatre objets :

```cpp
Motor motors[4];
```

Éviter de créer inutilement une couche supplémentaire si elle n'apporte rien.

---

## 3.6 `OmniDriveMixer`

Responsabilités :

- recevoir :
  - `translationX` ;
  - `translationY` ;
  - `rotationCommand` ;
- calculer les quatre commandes de roues ;
- utiliser les angles configurables ;
- normaliser les sorties ;
- appliquer les gains liés à la géométrie.

La partie mathématique doit être clairement isolée du pilotage matériel.

Cela permettra de tester le mixer indépendamment des BTS7960.

---

## 3.7 `SafetyManager`

Responsabilités possibles :

- perte SBUS ;
- failsafe ;
- timeout ;
- conditions d'armement ;
- désarmement ;
- autorisation du mode test moteur.

Si ce module reste très petit, il peut être regroupé avec `RobotController`.

L'important est que les règles de sécurité restent facilement identifiables et ne soient pas dispersées dans tout le projet.

---

## 3.8 `ConfigManager`

Responsabilités :

- valeurs par défaut ;
- chargement NVS ;
- sauvegarde NVS ;
- validation ;
- version de configuration ;
- import / export JSON.

Les structures de configuration doivent être centralisées et faciles à retrouver.

---

## 3.9 `WebInterface`

Responsabilités :

- Wi-Fi ;
- serveur HTTP ;
- WebSocket ou SSE ;
- API ;
- interface Web ;
- télémétrie ;
- modification de configuration ;
- logs Web ;
- mode test moteur.

Éviter de créer un fichier C++ distinct pour chaque page Web si cela n'est pas nécessaire.

Les ressources front-end peuvent rester séparées dans `data/`.

---

## 3.10 `Logger`

Le logger doit rester simple.

Il doit fournir :

- niveaux de logs ;
- sortie série ;
- buffer circulaire ;
- accès à l'interface Web.

Ne pas développer un framework de logging disproportionné par rapport au projet.

---

# 3.11 Critère pour créer un nouveau module

Créer un nouveau fichier/module seulement si au moins l'un des critères suivants est rempli :

- responsabilité clairement différente ;
- code difficile à comprendre dans son module actuel ;
- besoin de tester la fonctionnalité indépendamment ;
- taille du module devenue importante ;
- fonctionnalité suffisamment autonome.

Ne pas créer un module uniquement parce qu'une classe ou une fonction pourrait techniquement être placée dans un fichier séparé.

---

# 3.12 Objectif global de l'architecture

Le projet doit trouver un équilibre entre :

```text
Fichier unique géant
        ❌

Architecture claire en quelques modules
        ✅

Multiplication de dizaines de micro-modules
        ❌
```

Le lecteur doit pouvoir comprendre l'architecture générale du firmware en quelques minutes en regardant :

```text
main.cpp
robot_controller.*
sbus_receiver.*
motor_controller.*
omni_drive_mixer.*
web_interface.*
config_manager.*
```

---

# 4. Réception SBUS

## 4.1 Liaison série

Le récepteur utilise **SBUS**.

Prévoir une UART matérielle dédiée.

Paramètres usuels SBUS :

```text
100000 bauds
8 bits
parité paire
2 bits de stop
signal inversé
```

L'ESP32-S3 permet l'utilisation d'une UART avec inversion du signal RX. L'implémentation devra privilégier l'inversion matérielle/configurée par le périphérique UART plutôt qu'une solution logicielle lente.

Le GPIO utilisé pour RX doit être configurable.

---

## 4.2 Données SBUS

Décoder au minimum :

- 16 voies analogiques ;
- flag `frame lost` ;
- flag `failsafe`.

Conserver les valeurs SBUS brutes pour le diagnostic.

Prévoir une représentation normalisée :

```cpp
-1.0 ... 0.0 ... +1.0
```

---

# 5. Affectation des voies RC

L'affectation doit être configurable depuis l'interface Web.

Valeurs par défaut proposées :

| Fonction | Voie |
|---|---:|
| Translation gauche/droite `vx` | CH1 |
| Translation avant/arrière `vy` | CH2 |
| Rotation `omega` | CH4 |
| Armement | CH5 |
| Mode lent / rapide | CH6 |

Ne pas coder ces voies en dur ailleurs que dans la configuration par défaut.

Pour chaque axe prévoir :

- choix de la voie SBUS ;
- inversion ;
- minimum ;
- centre ;
- maximum ;
- deadband ;
- expo ;
- gain.

---

# 6. Traitement des commandes RC

Pipeline recommandé :

```text
SBUS brut
   ↓
Calibration min/centre/max
   ↓
Normalisation -1 → +1
   ↓
Deadband
   ↓
Expo
   ↓
Gain utilisateur
   ↓
vx / vy / omega
   ↓
Mixer omnidirectionnel
   ↓
Normalisation globale
   ↓
Calibration moteur
   ↓
Rampe / limitation
   ↓
BTS7960
```

---

# 7. Deadband

Prévoir un deadband configurable autour du neutre.

Exemple :

```text
±3 à ±8 %
```

Après deadband, remettre à l'échelle afin de conserver la pleine course.

Exemple conceptuel :

```cpp
if (abs(x) < deadband)
    x = 0;
else
    x = sign(x) * (abs(x) - deadband) / (1.0 - deadband);
```

---

# 8. Courbe exponentielle

Prévoir une expo configurable pour rendre le robot plus doux près du neutre.

Exemple :

```cpp
output = (1.0 - expo) * input + expo * input * input * input;
```

Valeur :

```text
expo = 0.0 → linéaire
expo = 1.0 → cubique
```

Prévoir une expo indépendante pour :

- translation ;
- rotation.

---

# 9. Géométrie du robot

## 9.1 Convention de coordonnées

Utiliser la convention :

```text
          +Y
           ↑
           |
      +X ← O → -X
           |
           ↓
          -Y
```

ou une convention différente si plus intuitive, mais :

- elle doit être documentée ;
- elle doit être utilisée de façon cohérente partout.

Recommandation :

```text
vx > 0 : déplacement vers la droite
vy > 0 : déplacement vers l'avant
omega > 0 : rotation antihoraire
```

---

## 9.2 Numérotation des moteurs

Définir clairement la position physique.

Exemple :

```text
              AVANT

        M0           M1


        M3           M2

             ARRIÈRE
```

Cette convention doit apparaître :

- dans le code ;
- dans la documentation ;
- dans l'interface Web.

---

# 10. Géométrie des roues et angle à 60°

Le robot utilise quatre roues omnidirectionnelles disposées avec une géométrie indiquée comme **60°**.

La signification physique exacte du 60° devra être confirmée lors de la mise au point :

- angle de l'axe de roue par rapport au châssis ;
- angle de direction de traction de la roue ;
- angle des rouleaux ;
- autre géométrie mécanique.

Pour cette raison, **ne pas coder un mixage figé**.

Le mixer doit utiliser une description paramétrique de chaque roue.

Pour chaque roue `i`, définir au minimum :

```cpp
struct WheelConfig {
    float driveAngleDeg;
    float rotationGain;
    float gain;
    bool inverted;
};
```

Le calcul de translation peut être basé sur la projection du vecteur de vitesse demandé sur la direction motrice de chaque roue.

Exemple générique :

```cpp
motor[i] =
    vx * cos(theta[i])
  + vy * sin(theta[i])
  + omega * rotationGain[i];
```

La convention exacte de `theta` doit être documentée.

Cette approche permet d'adapter rapidement le mixer à la géométrie réelle sans réécrire l'algorithme.

---

# 11. Mode de calibration géométrique

L'interface Web devra permettre d'éditer pour chaque moteur :

- angle de roue ;
- sens moteur ;
- gain moteur ;
- gain de rotation.

Prévoir également plusieurs presets éventuels :

- géométrie utilisateur ;
- configuration symétrique ;
- autres géométries utiles lors des essais.

Le preset actif ne doit pas empêcher l'édition manuelle.

---

# 12. Normalisation du mixage

Après calcul :

```cpp
m0
m1
m2
m3
```

chercher :

```cpp
maxAbs = max(abs(m0), abs(m1), abs(m2), abs(m3));
```

Si :

```cpp
maxAbs > 1.0
```

alors diviser toutes les sorties par `maxAbs`.

Cela permet de conserver la direction globale demandée sans saturer une seule roue.

---

# 13. Limitation de puissance

Prévoir :

```cpp
maxMotorOutput
```

configurable entre :

```text
0 → 100 %
```

Exemple :

```cpp
motorCommand *= maxMotorOutput;
```

Ajouter éventuellement :

- limite translation ;
- limite rotation ;
- profil lent ;
- profil rapide.

---

# 14. Rampe d'accélération

Une variation instantanée de :

```text
0 → 100 %
```

doit pouvoir être limitée.

Prévoir :

```cpp
maxAccelerationPerSecond
maxDecelerationPerSecond
```

La décélération pourra être différente de l'accélération.

Important :

> Une commande de sécurité / failsafe doit pouvoir contourner la rampe et forcer immédiatement les moteurs à zéro.

---

# 15. Pilotage des BTS7960

Chaque BTS7960 dispose typiquement de :

- `RPWM`
- `LPWM`
- `R_EN`
- `L_EN`

Selon le câblage retenu, les broches Enable peuvent être :

- commandées par l'ESP32 ;
- ou fixées à l'état actif matériellement.

La configuration devra indiquer ce choix.

---

## 15.1 Commande recommandée

Pour une consigne moteur :

```text
-1.0 → +1.0
```

### Marche positive

```text
RPWM = PWM
LPWM = 0
```

### Marche négative

```text
RPWM = 0
LPWM = PWM
```

### Arrêt

Définir explicitement le comportement attendu :

```text
RPWM = 0
LPWM = 0
```

Tester physiquement si cela correspond au comportement voulu avec les modules utilisés.

---

# 16. PWM

Utiliser le périphérique PWM matériel de l'ESP32.

Prévoir une configuration centralisée :

```cpp
PWM_FREQUENCY
PWM_RESOLUTION
```

Une fréquence de départ pourra être testée autour de :

```text
15 à 25 kHz
```

afin de limiter le bruit audible.

Cette valeur devra rester configurable dans le code.

---

# 17. GPIO

Ne pas disperser les GPIO dans le code.

Tous les GPIO doivent être regroupés dans un **fichier de pinout séparé**, par exemple :

```text
include/pinout.h
```

Aucun numéro de broche ne doit être codé en dur dans :

- `main.cpp`
- `motor_controller.cpp`
- `sbus_receiver.cpp`
- `web_interface.cpp`
- ou tout autre module métier

Exemple :

```cpp
struct MotorPins {
    int rightPwmPin;
    int leftPwmPin;
    int rightEnablePin;
    int leftEnablePin;
};
```

Puis :

```cpp
namespace Pinout
{
    constexpr int sbusRxPin = ...;
    constexpr int statusLedPin = ...;

    constexpr MotorPins motor0Pins = { ... };
    constexpr MotorPins motor1Pins = { ... };
    constexpr MotorPins motor2Pins = { ... };
    constexpr MotorPins motor3Pins = { ... };
}
```

Les broches définitives seront renseignées en fonction du câblage réel du robot et des contraintes de la carte Freenove ESP32-S3 WROOM.

Le fichier de pinout doit jouer le rôle de **référence unique de câblage**.

---

# 18. Sécurité / armement

Le robot ne doit jamais démarrer moteurs actifs immédiatement après boot.

États proposés :

```text
BOOT
WAITING_FOR_SBUS
DISARMED
ARMED
FAILSAFE
ERROR
```

---

## 18.1 Conditions d'armement

Pour passer en `ARMED` :

- SBUS valide ;
- aucune condition failsafe ;
- commandes de translation proches du neutre ;
- commande de rotation proche du neutre ;
- voie d'armement dans l'état attendu.

Une temporisation de validation, par exemple :

```text
500 ms
```

peut éviter les armements parasites.

---

## 18.2 Désarmement

Désarmement immédiat si :

- voie d'armement désactivée ;
- SBUS failsafe ;
- perte prolongée des trames ;
- erreur critique ;
- arrêt demandé depuis l'interface Web si cette fonction est activée.

---

# 19. Timeout SBUS

En plus du flag failsafe du protocole, utiliser un watchdog temporel.

Exemple :

```text
si aucune trame valide depuis 100 ms
→ arrêt moteur immédiat
→ état FAILSAFE
```

La valeur doit être configurable.

---

# 20. Wi-Fi

L'ESP32 doit héberger l'interface Web.

Le fonctionnement doit être possible **sans accès Internet**.

Mode par défaut recommandé :

## Access Point

L'ESP32 crée son propre réseau Wi-Fi :

```text
SSID : RobotRC-XXXX
```

Le suffixe peut être dérivé de l'identifiant de la puce.

Exemple d'adresse :

```text
192.168.4.1
```

L'utilisateur se connecte directement au robot depuis :

- téléphone ;
- tablette ;
- ordinateur portable.

Aucune connexion Internet ne doit être nécessaire pour charger l'interface.

Tous les fichiers CSS/JS doivent donc être hébergés localement.

---

# 21. Mode Wi-Fi optionnel

Prévoir éventuellement deux modes :

```text
AP
STA
```

### AP

Le robot crée son réseau.

### STA

Le robot rejoint un réseau existant.

Le mode AP doit rester disponible comme solution de secours afin de ne pas perdre l'accès au robot suite à une mauvaise configuration réseau.

---

# 22. Interface Web

L'interface doit être simple, responsive et utilisable sur smartphone.

Éviter les frameworks Web lourds nécessitant Internet.

Préférer :

- HTML ;
- CSS ;
- JavaScript vanilla ;
- éventuellement une petite librairie locale si réellement utile.

---

# 23. Pages / sections de l'interface

Une application monopage est acceptable.

Prévoir les sections suivantes.

---

## 23.1 Tableau de bord

Afficher en temps réel :

### Radio

- SBUS connecté / perdu ;
- failsafe ;
- frame lost ;
- fréquence approximative des trames ;
- âge de la dernière trame ;
- valeurs des voies utilisées.

### Commandes

Afficher :

```text
vx
vy
omega
```

sous forme :

- numérique ;
- jauge ;
- éventuellement joystick 2D.

### Moteurs

Pour chacun des quatre moteurs :

- consigne calculée ;
- consigne après limitation ;
- sens ;
- PWM ;
- inversion active ou non.

### Robot

- ARMED / DISARMED ;
- FAILSAFE ;
- uptime ;
- état Wi-Fi ;
- adresse IP ;
- mémoire libre ;
- fréquence de boucle.

---

# 24. Visualisation du mouvement

Ajouter une représentation graphique simple du robot vu de dessus.

Afficher :

- les 4 roues ;
- la numérotation M0 à M3 ;
- une flèche représentant `vx/vy` ;
- une indication de rotation `omega` ;
- la puissance demandée à chaque roue.

Ce graphique peut être réalisé en SVG directement dans la page.

---

# 25. Configuration RC

Créer une page ou section :

## Radio / SBUS

Paramètres modifiables :

- voie translation X ;
- voie translation Y ;
- voie rotation ;
- voie armement ;
- voie mode de vitesse ;
- inversion de chaque axe ;
- min ;
- centre ;
- max ;
- deadband ;
- expo ;
- gain.

Afficher les valeurs SBUS brutes en direct pour faciliter l'identification des voies.

---

# 26. Assistant de calibration SBUS

Prévoir si possible un petit assistant :

### Étape 1
mettre tous les manches au centre.

### Étape 2
placer les manches aux extrêmes.

### Étape 3
déterminer automatiquement :

- minimum ;
- centre ;
- maximum.

La validation finale reste manuelle.

---

# 27. Configuration moteurs

Pour chaque moteur :

```text
M0
M1
M2
M3
```

permettre :

- inversion du sens ;
- gain ;
- PWM maximum ;
- angle géométrique ;
- gain de rotation ;
- activation/désactivation pour diagnostic.

---

# 28. Test moteur depuis l'interface Web

Ajouter un mode de test moteur.

Ce mode doit être **fortement sécurisé**.

Conditions minimales :

- robot DISARMED ;
- SBUS non autorisé à commander simultanément ;
- confirmation explicite dans l'interface ;
- puissance limitée, par exemple à 10 ou 20 % ;
- durée maximale automatique ;
- arrêt immédiat si la connexion Web disparaît.

Permettre :

```text
M0 + faible puissance
M0 - faible puissance
M1 + faible puissance
...
```

Cette fonction sert uniquement à vérifier le câblage et le sens des roues.

---

# 29. Configuration de conduite

Paramètres :

- vitesse max translation ;
- vitesse max rotation ;
- expo translation ;
- expo rotation ;
- deadband ;
- accélération max ;
- décélération max ;
- mode lent ;
- mode rapide.

---

# 30. Configuration géométrique

Afficher une vue du robot avec les quatre roues.

Pour chaque roue :

- angle ;
- gain ;
- sens ;
- coefficient de rotation.

Permettre de modifier ces valeurs puis d'observer immédiatement les sorties théoriques du mixer.

---

# 31. Outil de test du mixer

Ajouter dans l'interface un mode de simulation ne commandant **pas** les moteurs.

Entrées :

```text
vx
vy
omega
```

Afficher les sorties :

```text
M0
M1
M2
M3
```

Permettre par exemple des boutons :

```text
Avant
Arrière
Gauche
Droite
Rotation gauche
Rotation droite
Diagonales
Stop
```

Cela permettra de valider mathématiquement le mixage avant les essais réels.

---

# 32. Logs

Créer un système centralisé de logs.

Niveaux :

```text
ERROR
WARN
INFO
DEBUG
TRACE
```

---

## 32.1 Sortie USB série

Les logs doivent être visibles dans le terminal série PlatformIO.

Exemple :

```text
[12.427][INFO][SBUS] Frame received
[12.431][INFO][ROBOT] ARMED
[18.722][WARN][SBUS] Frame timeout
[18.723][ERROR][SAFETY] Entering FAILSAFE
```

---

## 32.2 Logs Web

Conserver un buffer circulaire en RAM.

Exemple :

```text
100 à 500 dernières lignes
```

L'interface Web doit afficher ces logs sans recharger la page.

Préférer :

- WebSocket ;
- ou Server-Sent Events.

Ne pas utiliser du polling HTTP très rapide si une solution événementielle simple est disponible.

---

# 33. Données temps réel

Utiliser idéalement un canal WebSocket ou SSE pour diffuser :

```json
{
  "state": "ARMED",
  "sbus": {
    "connected": true,
    "failsafe": false,
    "frameLost": false
  },
  "input": {
    "vx": 0.22,
    "vy": 0.71,
    "omega": -0.10
  },
  "motors": [
    0.81,
    0.52,
    0.39,
    0.68
  ]
}
```

Ne pas envoyer nécessairement toutes les données à chaque boucle moteur.

Une fréquence Web autour de :

```text
10 à 20 Hz
```

est suffisante pour le suivi visuel.

La boucle moteur doit rester indépendante du rafraîchissement Web.

---

# 34. API HTTP

Prévoir une API simple.

Exemple :

```text
GET  /api/status
GET  /api/config
POST /api/config
POST /api/config/save
POST /api/config/reset
GET  /api/logs
POST /api/motor-test
POST /api/reboot
```

Les noms exacts peuvent évoluer.

---

# 35. Configuration persistante

Les paramètres doivent être stockés en NVS.

Les structures de configuration doivent être définies dans un **fichier de configuration séparé**, par exemple :

```text
include/robot_config.h
```

La logique de chargement / sauvegarde peut être implémentée dans :

```text
src/config_manager.cpp
```

Créer une structure versionnée :

```cpp
struct RobotConfiguration {
    uint32_t version;

    // Radio
    // Motors
    // Mixer
    // Safety
    // Wi-Fi
};
```

Au boot :

```text
config valide
    → chargement

config absente / incompatible
    → valeurs par défaut
```

Les valeurs par défaut doivent elles aussi être centralisées dans ce système de configuration, et non dispersées dans plusieurs modules.

---

# 36. Sauvegarde depuis le Web

Les modifications Web ne doivent pas forcément être écrites en flash à chaque mouvement d'un slider.

Prévoir :

```text
édition en RAM
→ bouton "Enregistrer"
→ écriture NVS
```

Cela évite les écritures flash inutiles.

---

# 37. Export / import de configuration

Ajouter si possible :

```text
Exporter configuration JSON
Importer configuration JSON
```

Exemple :

```json
{
  "version": 1,
  "radio": {},
  "motors": {},
  "mixer": {},
  "safety": {}
}
```

Cette fonction facilitera :

- sauvegarde ;
- duplication entre robots ;
- retour à une configuration connue.

---

# 38. Bouton Reset configuration

Prévoir une méthode pour revenir aux paramètres par défaut :

- depuis l'interface ;
- éventuellement via un bouton physique maintenu au boot.

Il faut éviter qu'une mauvaise configuration Wi-Fi rende définitivement l'interface inaccessible.

---

# 39. Boucle temps réel

La commande moteur ne doit pas dépendre du serveur Web.

Architecture souhaitée :

```text
Task / boucle temps réel
    ├── lecture SBUS
    ├── sécurité
    ├── calcul commandes
    ├── mixer
    └── moteurs

Task / boucle communication
    ├── Wi-Fi
    ├── Web
    ├── télémétrie
    └── logs
```

Il n'est pas obligatoire d'utiliser plusieurs tâches FreeRTOS dès la première version, mais l'architecture doit éviter qu'une requête Web bloque le pilotage moteur.

---

# 40. Fréquences recommandées

Ordres de grandeur :

### Contrôle robot

```text
100 à 500 Hz
```

selon contraintes.

### SBUS

Traiter chaque trame dès réception.

### Interface Web

```text
10 à 20 Hz
```

maximum nécessaire pour l'affichage courant.

### Logs

Éviter de générer des logs à chaque boucle de contrôle.

---

# 41. Machine d'états

Implémenter explicitement une machine d'états.

```cpp
enum class RobotState {
    BOOT,
    WAITING_FOR_RADIO,
    DISARMED,
    ARMED,
    FAILSAFE,
    MOTOR_TEST,
    ERROR
};
```

Les transitions doivent être centralisées et journalisées.

---

# 42. LED d'état

Si une LED utilisable est disponible, fournir des indications visuelles simples.

Exemple :

```text
clignotement lent     → DISARMED
fixe                   → ARMED
clignotement rapide    → FAILSAFE
double flash           → attente SBUS
```

Le GPIO et la polarité doivent rester configurables.

---

# 43. Démarrage

Séquence recommandée :

```text
1. Initialisation série
2. Chargement configuration
3. Initialisation GPIO
4. Mise à zéro moteurs
5. Initialisation SBUS
6. Initialisation Wi-Fi
7. Initialisation serveur Web
8. Attente radio valide
9. Passage DISARMED
10. Armement uniquement après validation explicite
```

Les moteurs doivent être forcés à zéro dès les premières instructions possibles.

---

# 44. Diagnostics série au démarrage

Afficher au minimum :

```text
Firmware version
Build date
ESP32 model
Configuration version
Wi-Fi mode
IP
GPIO moteurs
GPIO SBUS
PWM frequency
Mixer angles
SBUS detected/not detected
```

---

# 45. Page système / diagnostic

Afficher :

- version firmware ;
- uptime ;
- reset reason ;
- heap libre ;
- heap minimum ;
- Wi-Fi RSSI en mode station ;
- nombre de clients Web ;
- fréquence boucle contrôle ;
- nombre de trames SBUS reçues ;
- nombre de pertes de trames ;
- nombre d'entrées failsafe.

---

# 46. Sécurité de l'interface Web

L'interface Web est avant tout un outil local de configuration.

Prévoir au minimum :

- validation stricte des paramètres reçus ;
- aucune commande moteur arbitraire hors mode test ;
- impossibilité de modifier une valeur hors plage ;
- arrêt du test moteur si timeout ;
- désactivation du test moteur lors de l'armement RC.

Un mot de passe Wi-Fi AP doit pouvoir être configuré.

---

# 47. Priorité de commande

Priorité impérative :

```text
FAILSAFE
    ↓
ARRÊT / DISARM
    ↓
MOTOR TEST sécurisé
    ↓
Commande SBUS
```

Aucune fonction Web ne doit pouvoir contourner un failsafe.

---

# 48. Valeurs par défaut raisonnables

Créer des valeurs de départ mais les regrouper clairement.

Exemple :

```cpp
constexpr float DEFAULT_DEADBAND = 0.05f;
constexpr float DEFAULT_TRANSLATION_EXPO = 0.25f;
constexpr float DEFAULT_ROTATION_EXPO = 0.25f;
constexpr float DEFAULT_MAX_OUTPUT = 0.50f;
constexpr uint32_t DEFAULT_SBUS_TIMEOUT_MS = 100;
```

Le robot doit démarrer volontairement avec une puissance limitée pour les premiers essais.

---

# 49. Phase 1 — POC minimal

La première version doit uniquement valider la chaîne complète :

```text
SBUS
→ normalisation
→ mixer
→ sorties moteurs
→ failsafe
```

Fonctions nécessaires :

- décodage SBUS ;
- affichage voies dans le terminal série ;
- trois axes `vx`, `vy`, `omega` ;
- mixer 4 roues ;
- BTS7960 ;
- arrêt radio ;
- limitation de puissance.

---

# 50. Phase 2 — Interface Web de diagnostic

Ajouter :

- Wi-Fi AP ;
- page Web embarquée ;
- affichage SBUS ;
- affichage `vx/vy/omega` ;
- sorties moteurs ;
- état robot ;
- logs temps réel.

---

# 51. Phase 3 — Configuration Web

Ajouter :

- configuration voies ;
- inversions ;
- deadbands ;
- expo ;
- puissance ;
- rampes ;
- calibration moteurs ;
- paramètres de géométrie ;
- sauvegarde NVS.

---

# 52. Phase 4 — Outils de calibration

Ajouter :

- assistant calibration SBUS ;
- test individuel moteurs ;
- simulateur de mixer ;
- import/export JSON ;
- diagnostic système avancé.

---

# 53. Critères de validation

Le développement sera considéré fonctionnel lorsque les tests suivants seront réussis.

## Test 1 — SBUS

Chaque mouvement de manche est affiché correctement et de façon stable.

## Test 2 — Neutre

Manches au centre :

```text
M0 = 0
M1 = 0
M2 = 0
M3 = 0
```

## Test 3 — Translation avant

Les quatre roues produisent la combinaison attendue pour déplacer le robot vers l'avant.

## Test 4 — Translation latérale

Le robot peut se déplacer latéralement sans rotation significative.

## Test 5 — Rotation

Une commande de rotation produit une rotation sur place.

## Test 6 — Combinaison

Une translation et une rotation peuvent être appliquées simultanément.

## Test 7 — Saturation

Une commande combinée maximale ne génère jamais une sortie hors de :

```text
-1.0 ... +1.0
```

## Test 8 — Failsafe

Débrancher ou éteindre le récepteur :

```text
→ moteurs arrêtés immédiatement
→ état FAILSAFE
```

## Test 9 — Redémarrage

Après reset :

```text
→ aucune activation moteur automatique
```

## Test 10 — Configuration

Modifier un paramètre depuis le Web, enregistrer, redémarrer :

```text
→ paramètre conservé
```

## Test 11 — Interface sans Internet

Connexion directe au Wi-Fi du robot :

```text
→ interface totalement fonctionnelle sans Internet
```

---

# 54. Contraintes de qualité du code

Le code doit être écrit pour être **compréhensible et maintenable**, y compris par une personne encore peu familière avec l'ESP32, sans dégrader pour autant la qualité technique de l'implémentation.

Il doit :

- être lisible sans être simpliste ;
- utiliser les abstractions utiles lorsqu'elles améliorent réellement le code ;
- être commenté lorsque nécessaire ;
- documenter les modules et les fonctions publiques importantes ;
- utiliser des noms explicites ;
- conserver des noms de variables et fonctions en anglais, explicites et cohérents ;
- éviter les constantes magiques ;
- éviter les abréviations ambiguës ;
- éviter les mécanismes C++ complexes sans nécessité ;
- éviter les délais bloquants `delay()` dans la logique principale ;
- isoler le matériel du calcul mathématique ;
- permettre de tester le mixer indépendamment des moteurs ;
- centraliser la configuration ;
- être structuré pour faciliter l'ajout futur de capteurs ;
- limiter la taille et la responsabilité de chaque fichier ;
- conserver un `main.cpp` très lisible ;
- fournir des messages de diagnostic compréhensibles ;
- pouvoir être ouvert et compilé directement sous **VSCode + PlatformIO**.

Lorsqu'il existe plusieurs solutions techniques équivalentes, privilégier celle qui est la plus simple à comprendre et maintenir.

---

# 55. Évolutions futures à anticiper

Ne pas implémenter immédiatement, mais conserver une architecture compatible avec :

- encodeurs moteurs ;
- odométrie ;
- IMU ;
- correction de cap ;
- régulation PID vitesse moteur ;
- limitation courant ;
- mesure batterie ;
- télémétrie tension ;
- arrêt batterie faible ;
- capteurs de distance ;
- écran OLED ;
- buzzer ;
- éclairage LED ;
- OTA ;
- communication ESP-NOW ;
- pilotage autonome ou semi-autonome.

---

# 56. Mise à jour OTA

Une mise à jour OTA via l'interface Web pourra être ajoutée ultérieurement.

Si elle est implémentée :

- uniquement robot DISARMED ;
- moteurs forcés à zéro ;
- contrôle du type de fichier ;
- affichage progression ;
- reboot automatique après succès.

Ne pas faire de l'OTA une dépendance de la première version.

---

# 57. Informations restant à compléter lors de l'intégration matérielle

L'agent doit laisser clairement identifiables les paramètres suivants :

```text
[À DÉFINIR] GPIO SBUS RX

[À DÉFINIR] GPIO M0 RPWM
[À DÉFINIR] GPIO M0 LPWM
[À DÉFINIR] GPIO M0 REN
[À DÉFINIR] GPIO M0 LEN

[À DÉFINIR] GPIO M1 RPWM
[À DÉFINIR] GPIO M1 LPWM
[À DÉFINIR] GPIO M1 REN
[À DÉFINIR] GPIO M1 LEN

[À DÉFINIR] GPIO M2 RPWM
[À DÉFINIR] GPIO M2 LPWM
[À DÉFINIR] GPIO M2 REN
[À DÉFINIR] GPIO M2 LEN

[À DÉFINIR] GPIO M3 RPWM
[À DÉFINIR] GPIO M3 LPWM
[À DÉFINIR] GPIO M3 REN
[À DÉFINIR] GPIO M3 LEN
```

Si les `EN` sont câblés en permanence à l'état actif, ils peuvent ne pas consommer de GPIO.

À compléter également :

```text
[À DÉFINIR] angle exact de chaque roue
[À DÉFINIR] orientation physique M0/M1/M2/M3
[À DÉFINIR] sens positif électrique de chaque moteur
[À DÉFINIR] voies SBUS définitives
[À DÉFINIR] plage SBUS réelle du récepteur
```

---

# 58. Livrables demandés à l'agent

L'agent doit produire :

1. un projet PlatformIO compilable ;
2. un `README.md` ;
3. un schéma clair de l'architecture logicielle ;
4. une table des GPIO ;
5. un fichier de pinout séparé ;
6. un fichier de configuration séparé ;
7. les valeurs de configuration par défaut ;
8. le firmware ESP32 ;
9. les fichiers de l'interface Web ;
10. un système de configuration persistante ;
11. un système de logs ;
12. une procédure de premier démarrage ;
13. une procédure de calibration ;
14. une procédure de test des quatre moteurs ;
15. une procédure de test du failsafe ;
16. une description mathématique du mixer utilisé.

---

# 59. Point important pour l'agent

Ne pas supposer qu'une formule standard de robot Mecanum ou de plateforme X-drive correspond automatiquement à cette mécanique.

La géométrie réelle est :

> **robot omnidirectionnel 4 roues, roues indiquées comme positionnées à 60°**

Le mixage doit donc rester **paramétrique** jusqu'à validation physique.

La première priorité est de permettre une calibration rapide des angles et signes depuis l'interface Web.

---

# 60. Résultat attendu

À terme, l'utilisateur doit pouvoir :

1. mettre le robot sous tension ;
2. se connecter au Wi-Fi émis par l'ESP32 ;
3. ouvrir l'interface locale ;
4. vérifier immédiatement la réception SBUS ;
5. visualiser les manches et les sorties moteurs ;
6. configurer les voies et inversions ;
7. régler le comportement de conduite ;
8. calibrer la géométrie des roues ;
9. tester les moteurs individuellement ;
10. enregistrer la configuration ;
11. armer le robot depuis la radiocommande ;
12. piloter le robot en translation et rotation ;
13. consulter les logs et états en cas de problème.

La sécurité du robot et la perte radio doivent rester prioritaires sur toutes les autres fonctions.
