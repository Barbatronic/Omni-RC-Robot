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
 * ---------------------------------------------------------------------------
 * Deux cibles materielles
 * ---------------------------------------------------------------------------
 *
 *   TARGET_ESP32S3_ZERO defini   carte definitive, ESP32-S3-Zero + PCB
 *   sinon                        prototype sur Freenove ESP32-S3 WROOM
 *
 * Le drapeau est pose par l'environnement PlatformIO. Les deux brochages sont
 * conserves cote a cote : le prototype doit rester flashable tant que la carte
 * definitive n'est pas validee.
 */

#include <cstdint>

/**
 * @brief Broches PWM d'un driver BTS7960 / IBT-2.
 *
 * RPWM / LPWM recoivent le signal PWM, un seul des deux etant actif a la fois.
 * Les entrees Enable des drivers ne figurent pas ici : sur la carte definitive
 * elles sont toutes reliees a une broche unique, decrite par Pinout::motorEnablePin.
 */
struct MotorPins
{
    int rightPwmPin;  ///< RPWM : PWM du sens "avant" du moteur
    int leftPwmPin;   ///< LPWM : PWM du sens "arriere" du moteur
};

namespace Pinout
{
#if defined(TARGET_ESP32S3_ZERO)

    // =========================================================================
    // Carte definitive : ESP32-S3-Zero (ESP32-S3FH4R2, 4 Mo flash, 2 Mo PSRAM)
    // Brochage conforme au schema KiCad du PCB.
    //
    // Broches non disponibles sur ce module :
    //   GPIO 26 ... 32   flash interne
    //   GPIO 33 ... 37   non sorties (reservees PSRAM octale)
    //   GPIO 19 / 20     USB natif, seul lien avec le PC
    //   GPIO 0 / 45 / 46 strapping (0 = bouton BOOT)
    //   GPIO 3           strapping JTAG, laisse non connecte sur le PCB
    // =========================================================================

    /// Entree SBUS / iBUS du recepteur RC, sur GP44 (U0RXD).
    ///
    /// Utilisable ici parce que le S3-Zero n'embarque aucun pont USB-UART : la
    /// console serie passe par l'USB natif, ce qui libere GP43 et GP44. Le
    /// firmware route une UART libre sur cette broche via la matrice GPIO, il
    /// n'utilise donc pas UART0 lui-meme.
    constexpr int radioRxPin = 44;

    /// UART materielle dediee a la radio (1 ou 2 ; 0 reste au bootloader ROM).
    constexpr int radioUartNumber = 1;

    // -------------------------------------------------------------------------
    // Moteurs
    //
    //               AVANT
    //          M0           M1
    //          M3           M2
    //              ARRIERE
    //
    // Correspondance avec les reperes du schema :
    //   Motor_BTS7960_01 -> M0     Motor_BTS7960_03 -> M2
    //   Motor_BTS7960_02 -> M1     Motor_BTS7960_04 -> M3
    //
    // ATTENTION : l'ESP32-S3 ne dispose que de 8 canaux LEDC (PWM materiel).
    // Les 4 moteurs x 2 broches PWM les consomment en totalite. Aucun autre
    // peripherique du firmware ne doit appeler ledcSetup().
    // -------------------------------------------------------------------------

    constexpr MotorPins motor0Pins = {  1,  2 };  ///< M0 avant gauche  (BTS7960_01)
    constexpr MotorPins motor1Pins = {  4,  5 };  ///< M1 avant droit   (BTS7960_02)
    constexpr MotorPins motor2Pins = {  6, 12 };  ///< M2 arriere droit (BTS7960_03)
    constexpr MotorPins motor3Pins = { 11, 10 };  ///< M3 arriere gauche(BTS7960_04)

    /// Validation commune aux quatre drivers (R_EN et L_EN relies ensemble).
    ///
    /// Une resistance de rappel de 10 kohm la maintient a l'etat bas sur le PCB :
    /// au reset, pendant un televersement ou tant que le firmware n'a pas repris
    /// la main, les ponts en H sont donc desactives par le materiel. Le firmware
    /// ne la leve que lorsque les moteurs sont reellement autorises, ce qui donne
    /// une coupure materielle en plus de la mise a zero du PWM.
    constexpr int motorEnablePin = 7;

    /// LED RGB adressable WS2812 embarquee sur le module.
    constexpr int  statusLedPin           = 21;
    constexpr bool statusLedIsAddressable = true;
    constexpr bool statusLedIsActiveLow   = false;

    /// Bus I2C sorti sur les connecteurs J18 et J19, avec rappels de 10 kohm.
    /// Reserve aux capteurs a venir (centrale inertielle, mesure batterie...).
    constexpr int i2cSdaPin = 8;
    constexpr int i2cSclPin = 9;

    /// Bouton maintenu au demarrage pour restaurer la configuration par defaut.
    constexpr int  configResetButtonPin         = 0;
    constexpr bool configResetButtonIsActiveLow = true;

#else

    // =========================================================================
    // Prototype : Freenove ESP32-S3 WROOM (N8R8, PSRAM octale)
    //
    // Broches a NE PAS utiliser sur cette carte :
    //   GPIO 19 / 20        USB natif (D- / D+)
    //   GPIO 26 ... 37      flash QSPI et PSRAM octale
    //   GPIO 43 / 44        UART0, utilise par le terminal serie
    //   GPIO 0 / 45 / 46    strapping (0 = bouton BOOT)
    //   GPIO 38 / 39 / 40   lecteur de carte microSD
    //   GPIO 48             LED RGB embarquee
    //
    // Connecteur camera (occupe des qu'un module est branche) :
    //   GPIO 4 5 6 7 8 9 10 11 12 13 15 16 17 18
    //   GPIO 4 (SIOD) et 5 (SIOC) portent le bus I2C du capteur : un PWM y est
    //   interprete comme du trafic I2C et la commande moteur est corrompue.
    //   C'est pourquoi M0 est sur GPIO 1 et 2 sur ce prototype.
    //
    // Broches reellement libres : 1, 2, 14, 21, 41, 42, 47.
    // =========================================================================

    constexpr int radioRxPin      = 18;
    constexpr int radioUartNumber = 1;

    constexpr MotorPins motor0Pins = {  1,  2 };  ///< M0 avant gauche
    constexpr MotorPins motor1Pins = {  6,  7 };  ///< M1 avant droit
    constexpr MotorPins motor2Pins = { 15, 16 };  ///< M2 arriere droit
    constexpr MotorPins motor3Pins = { 17,  8 };  ///< M3 arriere gauche

    /// Les entrees Enable des IBT-2 sont cablees en dur sur +5 V sur ce
    /// prototype : le firmware ne les pilote pas.
    constexpr int motorEnablePin = -1;

    constexpr int  statusLedPin           = 48;
    constexpr bool statusLedIsAddressable = true;
    constexpr bool statusLedIsActiveLow   = false;

    constexpr int i2cSdaPin = -1;
    constexpr int i2cSclPin = -1;

    constexpr int  configResetButtonPin         = 0;
    constexpr bool configResetButtonIsActiveLow = true;

#endif

    constexpr MotorPins motorPins[4] = {
        motor0Pins,
        motor1Pins,
        motor2Pins,
        motor3Pins,
    };
}
