#pragma once

/**
 * @file pinout.h
 *
 * @brief Reference unique du cablage materiel du robot.
 *
 * Ce fichier regroupe TOUS les GPIO utilises par le firmware.
 * Aucun numero de broche ne doit apparaitre ailleurs dans le projet :
 * pour modifier le cablage, ce fichier doit etre le seul a editer.
 *
 * Cible : Freenove ESP32-S3 WROOM (N8R8, PSRAM octale).
 *
 * Broches a NE PAS utiliser sur cette carte :
 *   GPIO 19 / 20        USB natif (D- / D+)
 *   GPIO 26 ... 37      flash QSPI et PSRAM octale
 *   GPIO 43 / 44        UART0, utilise par le terminal serie
 *   GPIO 0 / 45 / 46    broches de strapping (0 = bouton BOOT, lisible apres demarrage)
 *   GPIO 38 / 39 / 40   lecteur de carte microSD de la carte
 *   GPIO 48             LED RGB adressable embarquee
 *
 * Connecteur camera (a considerer comme occupe si un module est branche) :
 *   GPIO 4 5 6 7 8 9 10 11 12 13 15 16 17 18
 *
 *   Toutes ne se valent pas. GPIO 4 (SIOD) et GPIO 5 (SIOC) portent le bus I2C
 *   du capteur : resistances de tirage sur la carte et puce toujours a l'ecoute.
 *   Un signal PWM y est interprete comme du trafic I2C et le capteur tire la
 *   ligne, ce qui corrompt la commande moteur. Ces deux broches sont donc
 *   INUTILISABLES des qu'une camera est connectee.
 *
 *   Les autres broches camera sont des sorties de donnees que le capteur ne
 *   pilote pas tant qu'il n'a pas ete configure par I2C : elles restent
 *   utilisables, ce qui est verifie sur ce robot pour M1, M2, M3 et la radio.
 *
 * Broches reellement libres de tout usage : 1, 2, 14, 21, 41, 42, 47.
 *
 * Les broches choisies ci-dessous sont des valeurs de depart coherentes avec
 * la carte, mais elles doivent etre confirmees avec le cablage reel du robot.
 */

#include <cstdint>

/**
 * @brief Broches d'un driver BTS7960 / IBT-2.
 *
 * RPWM / LPWM recoivent le signal PWM (un seul des deux est actif a la fois).
 * REN / LEN sont les entrees "Enable" du driver. Si elles sont cablees en dur
 * sur +5 V, mettre -1 : le firmware ne les pilotera pas et n'occupera aucun GPIO.
 */
struct MotorPins
{
    int rightPwmPin;     ///< RPWM : PWM du sens "avant" du moteur
    int leftPwmPin;      ///< LPWM : PWM du sens "arriere" du moteur
    int rightEnablePin;  ///< R_EN, ou -1 si cable en dur a l'etat actif
    int leftEnablePin;   ///< L_EN, ou -1 si cable en dur a l'etat actif
};

namespace Pinout
{
    // -------------------------------------------------------------------------
    // Radiocommande
    // -------------------------------------------------------------------------

    /// Entree serie du recepteur RC (sortie SBUS ou iBUS selon le recepteur).
    /// La vitesse, le format et l'inversion du signal sont deduits du protocole
    /// et geres par l'UART : aucun inverseur externe n'est necessaire.
    /// [A CONFIRMER avec le cablage reel]
    constexpr int radioRxPin = 18;

    /// UART materielle dediee a la radio (1 ou 2 ; 0 est reserve au terminal serie).
    constexpr int radioUartNumber = 1;

    // -------------------------------------------------------------------------
    // Moteurs
    //
    //               AVANT
    //          M0           M1
    //          M3           M2
    //              ARRIERE
    //
    // ATTENTION : l'ESP32-S3 ne dispose que de 8 canaux LEDC (PWM materiel).
    // Les 4 moteurs x 2 broches PWM les consomment donc en totalite.
    // Aucun autre peripherique du firmware ne doit utiliser ledcSetup().
    // [A CONFIRMER avec le cablage reel]
    // -------------------------------------------------------------------------

    /// M0 utilise deux broches totalement libres : les GPIO 4 et 5 d'origine
    /// sont le bus I2C de la camera et rendaient ce moteur inutilisable.
    constexpr MotorPins motor0Pins = { 1,  2,  -1, -1 };  ///< M0 avant gauche
    constexpr MotorPins motor1Pins = { 6,  7,  -1, -1 };  ///< M1 avant droit
    constexpr MotorPins motor2Pins = { 15, 16, -1, -1 };  ///< M2 arriere droit
    constexpr MotorPins motor3Pins = { 17, 8,  -1, -1 };  ///< M3 arriere gauche

    constexpr MotorPins motorPins[4] = {
        motor0Pins,
        motor1Pins,
        motor2Pins,
        motor3Pins,
    };

    // -------------------------------------------------------------------------
    // Entrees / sorties systeme
    // -------------------------------------------------------------------------

    /// LED de statut. Sur la Freenove ESP32-S3 WROOM, la LED embarquee est une
    /// LED RGB adressable (WS2812) sur GPIO 48.
    constexpr int  statusLedPin = 48;

    /// true  : LED adressable WS2812 pilotee par neopixelWrite()
    /// false : LED classique pilotee par digitalWrite()
    constexpr bool statusLedIsAddressable = true;

    /// Niveau logique qui allume une LED classique (ignore si adressable).
    constexpr bool statusLedIsActiveLow = false;

    /// Bouton maintenu au demarrage pour restaurer la configuration par defaut.
    /// GPIO 0 = bouton BOOT de la carte. Mettre -1 pour desactiver la fonction.
    constexpr int  configResetButtonPin       = 0;
    constexpr bool configResetButtonIsActiveLow = true;
}
