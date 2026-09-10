#pragma once

/**
 * @file robot_config.h
 *
 * @brief Structures de configuration du robot et valeurs par defaut.
 *
 * Ce fichier est volontairement distinct de pinout.h :
 *
 *   pinout.h        -> cablage materiel (quels GPIO)
 *   robot_config.h  -> parametres reglables par l'utilisateur (comment on pilote)
 *
 * Toute la configuration modifiable depuis l'interface Web est decrite ici, avec
 * ses valeurs par defaut et ses bornes de validation. Le chargement, la
 * sauvegarde en NVS et la validation sont implementes dans config_manager.cpp.
 *
 * Ce fichier n'inclut volontairement pas <Arduino.h> : il reste compilable sur PC,
 * ce qui permet de tester le mixer sans carte (voir test/test_mixer).
 */

#include <cstdint>

/// Nombre de moteurs / roues du robot.
constexpr int wheelCount = 4;

/// Longueur maximale (avec le zero terminal) d'un SSID ou d'un mot de passe Wi-Fi.
constexpr int wifiSsidBufferSize     = 33;
constexpr int wifiPasswordBufferSize = 65;

/**
 * @brief Version du format de configuration stocke en NVS.
 *
 * A incrementer des qu'un champ est ajoute, supprime ou change de signification :
 * une configuration enregistree avec une autre version est ignoree et remplacee
 * par les valeurs par defaut.
 */
/// Version 2 : ajout du protocole radio et de ses plages de calibration.
constexpr uint32_t robotConfigurationVersion = 2;

/**
 * @brief Protocole serie emis par le recepteur RC.
 *
 * Les deux protocoles different par la vitesse, le format et la polarite du
 * signal, ainsi que par la plage des valeurs de voies : le SBUS travaille en
 * unites arbitraires (172..1811), l'iBUS directement en microsecondes
 * (1000..2000). La calibration des axes depend donc du protocole.
 */
enum class RadioProtocol : uint8_t
{
    Sbus = 0,   ///< Futaba, FrSky : 100000 8E2 inverse
    Ibus = 1,   ///< FlySky (FS-iA6B, FS-iA10B) : 115200 8N1 non inverse
};

// =============================================================================
// Radiocommande
// =============================================================================

/**
 * @brief Calibration et traitement d'un axe de la radiocommande.
 *
 * Le pipeline applique, dans l'ordre :
 * calibration min/centre/max -> normalisation -1..+1 -> inversion -> deadband
 * -> expo -> gain.
 */
struct AxisConfiguration
{
    int   channelIndex;   ///< Voie radio utilisee, indexee a partir de 0 (CH1 -> 0)
    bool  inverted;       ///< Inverse le sens de l'axe
    int   rawMinimum;     ///< Valeur brute en butee basse
    int   rawCenter;      ///< Valeur brute au neutre
    int   rawMaximum;     ///< Valeur brute en butee haute
    float deadband;       ///< Zone morte autour du neutre, en fraction de course (0..0.5)
    float expo;           ///< 0 = lineaire, 1 = cubique
    float gain;           ///< Gain applique apres l'expo (0..1)
};

/**
 * @brief Affectation des voies radio et traitement des trois axes de pilotage.
 */
struct RadioConfiguration
{
    AxisConfiguration translationX;  ///< Translation laterale (vx)
    AxisConfiguration translationY;  ///< Translation longitudinale (vy)
    AxisConfiguration rotation;      ///< Rotation (omega)

    int  armChannelIndex;            ///< Voie de l'interrupteur d'armement
    int  armThresholdRawValue;       ///< Au-dessus de ce seuil brut, l'armement est demande
    bool armChannelIsInverted;       ///< Inverse la logique de la voie d'armement

    int  speedModeChannelIndex;      ///< Voie du selecteur lent / rapide, -1 pour desactiver
    int  speedModeThresholdRawValue; ///< Au-dessus de ce seuil brut, le mode rapide est actif

    uint32_t frameTimeoutMs;         ///< Duree sans trame valide declenchant le FAILSAFE

    RadioProtocol protocol;          ///< Protocole du recepteur
    bool     autoDetectProtocol;     ///< Reconnaitre le protocole au demarrage
};

// =============================================================================
// Moteurs et geometrie des roues
// =============================================================================

/**
 * @brief Description d'une roue pour le mixer et son moteur.
 *
 * @c driveAngleDeg est l'angle, en degres, de la direction motrice de la roue,
 * mesure dans le sens antihoraire depuis l'axe +X (voir omni_drive_mixer.h).
 * C'est le seul parametre a ajuster pour s'adapter a la geometrie reelle.
 */
struct WheelConfiguration
{
    float driveAngleDeg;  ///< Direction motrice de la roue, en degres depuis +X
    float rotationGain;   ///< Contribution de omega a cette roue (typiquement +1)
    float outputGain;     ///< Gain individuel du moteur (0..1), pour equilibrer les moteurs
    bool  inverted;       ///< Inverse le sens electrique du moteur
    bool  enabled;        ///< false : moteur force a zero (diagnostic)
};

/**
 * @brief Limites de puissance et de dynamique communes aux quatre moteurs.
 */
struct DriveConfiguration
{
    float maxMotorOutput;             ///< Plafond global de sortie moteur (0..1)

    /// Sortie minimale d'un moteur en mouvement, pour franchir son seuil de
    /// demarrage. Un moteur a courant continu avec reducteur ne demarre pas en
    /// dessous d'un certain rapport cyclique : il chauffe et bourdonne sans
    /// tourner. Toute commande non nulle est donc remontee a cette valeur, la
    /// pleine echelle restant inchangee.
    /// 0 desactive la compensation. Valeur typique : 0.15 a 0.25.
    float minMotorOutput;

    /// Exploite toute la capacite moteur quelle que soit la direction.
    ///
    /// La projection geometrique fait qu'une commande de translation ne charge
    /// pas les roues de la meme facon selon sa direction : avec des roues a
    /// 60 degres, une marche avant ne sollicite les moteurs qu'a la moitie de
    /// leur capacite, contre 87 % en lateral.
    ///
    /// Actif, ce reglage applique un facteur de rattrapage calcule a partir de
    /// la DIRECTION demandee, jamais de son amplitude : un plein manche donne la
    /// pleine echelle dans toutes les directions, et un petit mouvement de
    /// manche reste un petit mouvement. La proportionnalite manche / vitesse est
    /// preservee, seule la vitesse maximale change selon la direction.
    ///
    /// Inactif, la commande reste la projection geometrique brute : la vitesse
    /// est directement proportionnelle a la consigne, mais une partie de la
    /// capacite moteur n'est jamais utilisee.
    bool normalizeTranslation;
    float maxTranslationOutput;       ///< Plafond specifique a la translation (0..1)
    float maxRotationOutput;          ///< Plafond specifique a la rotation (0..1)
    float slowModeScale;              ///< Facteur applique en mode lent (0..1)
    float maxAccelerationPerSecond;   ///< Variation maximale en montee, par seconde
    float maxDecelerationPerSecond;   ///< Variation maximale en descente, par seconde
    uint32_t pwmFrequencyHz;          ///< Frequence PWM des BTS7960 (15000..25000 Hz conseille)
    uint8_t  pwmResolutionBits;       ///< Resolution PWM ; doit verifier 80 MHz / f >= 2^bits
};

// =============================================================================
// Securite
// =============================================================================

/**
 * @brief Regles d'armement et de test moteur.
 */
struct SafetyConfiguration
{
    uint32_t armingHoldMs;          ///< Duree pendant laquelle les conditions doivent tenir
    float    armingNeutralTolerance;///< Tolerance sur les manches au neutre pour armer (0..1)
    float    motorTestMaxOutput;    ///< Puissance maximale du test moteur Web (0..0.5)
    uint32_t motorTestTimeoutMs;    ///< Arret automatique du test moteur
};

// =============================================================================
// Wi-Fi
// =============================================================================

/**
 * @brief Mode reseau du robot.
 *
 * AccessPoint reste toujours disponible en secours : si la connexion en mode
 * Station echoue, le firmware bascule automatiquement en point d'acces afin de
 * ne jamais rendre l'interface inaccessible.
 */
enum class WifiMode : uint8_t
{
    AccessPoint = 0,
    Station     = 1,
};

/**
 * @brief Parametres reseau.
 *
 * Les identifiants du reseau d'infrastructure (mode Station) ne sont JAMAIS
 * ecrits en dur dans ce fichier : ils proviennent de include/wifi_secrets.h,
 * qui est exclu du depot Git. Voir wifi_secrets.example.h.
 */
struct WifiConfiguration
{
    WifiMode mode;
    char     accessPointSsid[wifiSsidBufferSize];         ///< Vide : genere depuis l'ID de puce
    char     accessPointPassword[wifiPasswordBufferSize]; ///< Vide : point d'acces ouvert
    char     stationSsid[wifiSsidBufferSize];
    char     stationPassword[wifiPasswordBufferSize];
    uint32_t stationConnectTimeoutMs;                     ///< Delai avant repli en point d'acces
};

// =============================================================================
// Configuration complete
// =============================================================================

/**
 * @brief Configuration persistante complete du robot.
 *
 * Cette structure est sauvegardee telle quelle en NVS, precedee de sa version.
 */
struct RobotConfiguration
{
    uint32_t            version;
    RadioConfiguration  radio;
    WheelConfiguration  wheels[wheelCount];
    DriveConfiguration  drive;
    SafetyConfiguration safety;
    WifiConfiguration   wifi;
};

// =============================================================================
// Valeurs par defaut
//
// Le robot demarre volontairement avec une puissance limitee : les premiers
// essais doivent pouvoir etre faits sans risque, roues levees.
// =============================================================================

namespace ConfigDefaults
{
    constexpr float    deadband              = 0.05f;
    constexpr float    translationExpo       = 0.25f;
    constexpr float    rotationExpo          = 0.25f;
    constexpr float    maxMotorOutput        = 0.50f;

    /// Compensation du seuil de demarrage desactivee par defaut : sa valeur
    /// depend des moteurs et du reducteur, elle se mesure sur le robot reel
    /// avec le test moteur de l'interface Web.
    constexpr float    minMotorOutput        = 0.0f;
    constexpr uint32_t sbusTimeoutMs         = 100;

    /// Plage brute typique d'un recepteur SBUS (172 .. 1811, neutre 992).
    constexpr int sbusRawMinimum = 172;
    constexpr int sbusRawCenter  = 992;
    constexpr int sbusRawMaximum = 1811;

    /// Plage brute typique d'un recepteur iBUS, en microsecondes.
    constexpr int ibusRawMinimum = 1000;
    constexpr int ibusRawCenter  = 1500;
    constexpr int ibusRawMaximum = 2000;

    /// Angle de la geometrie du robot, mesure depuis l'axe longitudinal.
    /// 45 degres correspond a un X-drive classique, 60 degres a ce robot.
    /// A confirmer lors de la mise au point mecanique.
    constexpr float wheelGeometryAngleDeg = 60.0f;

    constexpr uint32_t pwmFrequencyHz    = 20000;

    /// 10 bits (1024 pas) est la resolution maximale acceptee par le LEDC de
    /// l'ESP32-S3 a 20 kHz. Une valeur superieure serait refusee par le
    /// materiel ; MotorController retomberait alors automatiquement ici.
    constexpr uint8_t  pwmResolutionBits = 10;
}

/**
 * @brief Remplit une configuration avec les valeurs par defaut.
 *
 * Les identifiants Wi-Fi compiles (wifi_secrets.h) ne sont pas appliques ici :
 * ils sont ajoutes par ConfigManager, qui est le seul module a les connaitre.
 *
 * @param configuration Structure a initialiser.
 */
void applyDefaultConfiguration(RobotConfiguration& configuration);

/**
 * @brief Applique a tous les axes les plages de calibration d'un protocole.
 *
 * Les valeurs brutes SBUS et iBUS n'ont ni la meme echelle ni le meme neutre :
 * une calibration faite pour l'un n'a aucun sens pour l'autre. Cette fonction
 * remet les trois axes sur les valeurs typiques du protocole indique.
 *
 * @param configuration Configuration a mettre a jour.
 * @param protocol Protocole dont on veut les plages par defaut.
 */
void applyProtocolCalibrationDefaults(RobotConfiguration& configuration, RadioProtocol protocol);

/**
 * @brief Corrige toute valeur hors plage d'une configuration.
 *
 * Utilise apres un chargement NVS et apres toute modification recue de
 * l'interface Web : aucune valeur invalide ne doit atteindre les moteurs.
 *
 * @param configuration Structure a valider et corriger sur place.
 * @return true si au moins une valeur a du etre corrigee.
 */
bool clampConfigurationToValidRange(RobotConfiguration& configuration);
