#pragma once

/**
 * @file motor_controller.h
 *
 * @brief Pilotage des quatre moteurs a courant continu via des drivers BTS7960.
 *
 * Entrees  : consignes normalisees -1..+1 issues du mixer.
 * Sorties  : signaux PWM materiels (LEDC) sur les broches RPWM / LPWM.
 * Dialogue avec : RobotController.
 *
 * Contraintes materielles importantes :
 *
 *  - Un BTS7960 recoit un PWM sur RPWM ou sur LPWM, jamais sur les deux en meme
 *    temps : mettre les deux a l'etat haut mettrait le pont en court-circuit.
 *    Le code garantit toujours qu'une des deux sorties est a zero.
 *
 *  - L'ESP32-S3 ne possede que 8 canaux LEDC. Les 4 moteurs x 2 broches PWM les
 *    utilisent tous : aucun autre module ne doit appeler ledcSetup().
 *
 *  - A l'arret, RPWM = LPWM = 0. Le moteur est alors en roue libre.
 *
 *  - Sur la carte definitive, les entrees Enable des quatre drivers sont
 *    reliees a une broche unique munie d'une resistance de rappel au 0 V. Le
 *    firmware ne la leve que lorsque les moteurs sont autorises : les ponts en
 *    H sont donc coupes au niveau materiel des que le robot n'est ni arme ni en
 *    test, et pendant tout le demarrage. C'est une securite independante de la
 *    commande PWM, qui reste active meme si le logiciel se fige.
 */

#include <Arduino.h>

#include "pinout.h"
#include "robot_config.h"

/**
 * @brief Un moteur et son driver BTS7960.
 */
class Motor
{
public:
    /**
     * @brief Configure les broches et les canaux PWM materiels du moteur.
     *
     * @param pins Broches du driver.
     * @param rightPwmChannel Canal LEDC affecte a RPWM.
     * @param leftPwmChannel Canal LEDC affecte a LPWM.
     * @param pwmFrequencyHz Frequence PWM.
     * @param pwmResolutionBits Resolution PWM.
     * @return true si les deux canaux PWM ont ete acceptes par le materiel.
     */
    bool begin(const MotorPins& pins,
               uint8_t rightPwmChannel,
               uint8_t leftPwmChannel,
               uint32_t pwmFrequencyHz,
               uint8_t pwmResolutionBits);

    /**
     * @brief Applique une consigne au moteur.
     *
     * @param output Consigne entre -1.0 et +1.0. Le signe donne le sens.
     */
    void setOutput(float output);

    /// Coupe immediatement le moteur : RPWM = LPWM = 0.
    void stop();

    /// Derniere consigne appliquee, entre -1.0 et +1.0.
    float appliedOutput() const { return lastAppliedOutput; }

    /// Rapport cyclique brut envoye au driver, pour le diagnostic.
    uint32_t appliedDutyCycle() const { return lastAppliedDutyCycle; }

private:
    MotorPins pins                 = { -1, -1 };
    uint8_t   rightPwmChannel      = 0;
    uint8_t   leftPwmChannel       = 0;
    uint32_t  maximumDutyCycle     = 0;
    float     lastAppliedOutput    = 0.0f;
    uint32_t  lastAppliedDutyCycle = 0;
};

/**
 * @brief Gere les quatre moteurs du robot et la rampe d'acceleration.
 */
class MotorController
{
public:
    /**
     * @brief Initialise les quatre moteurs, sorties forcees a zero.
     *
     * @param drive Parametres PWM et limites de dynamique.
     */
    void begin(const DriveConfiguration& drive);

    /**
     * @brief Met a jour les parametres de pilotage.
     *
     * Si la frequence ou la resolution PWM ont change, le peripherique est
     * reconfigure immediatement, moteurs coupes au prealable. Cela permet de
     * chercher la bonne frequence depuis l'interface Web sans redemarrer : les
     * drivers BTS7960 commutent lentement et perdent nettement en rendement
     * au-dela d'une dizaine de kilohertz.
     */
    void setDriveConfiguration(const DriveConfiguration& drive);

    /**
     * @brief Applique les consignes des quatre moteurs en respectant la rampe.
     *
     * @param targets Consignes visees, entre -1.0 et +1.0.
     * @param deltaTimeSeconds Temps ecoule depuis l'appel precedent.
     */
    void applyOutputs(const float (&targets)[wheelCount], float deltaTimeSeconds);

    /**
     * @brief Arrete immediatement les quatre moteurs, sans passer par la rampe.
     *
     * Utilise par le failsafe et le desarmement : une commande de securite ne
     * doit jamais etre ralentie par la limitation d'acceleration.
     */
    void stopAllMotors();

    /// Consigne reellement appliquee au moteur @p motorIndex.
    float appliedOutput(int motorIndex) const;

    /// Rapport cyclique brut applique au moteur @p motorIndex.
    uint32_t appliedDutyCycle(int motorIndex) const;

private:
    /// Rapproche @p current de @p target en respectant les pentes autorisees.
    float applyRateLimit(float current, float target, float deltaTimeSeconds) const;

    /// Configure les huit canaux PWM materiels pour la frequence demandee.
    void configurePwmChannels(const DriveConfiguration& drive);

    /**
     * @brief Autorise ou coupe les ponts en H au niveau materiel.
     *
     * Sans effet si la carte ne cable pas de broche de validation commune.
     *
     * @param isEnabled true pour autoriser les drivers a conduire.
     */
    void setPowerStageEnabled(bool isEnabled);

    bool isPowerStageEnabled = false;

    Motor              motors[wheelCount];
    DriveConfiguration driveConfiguration{};
    float              currentOutputs[wheelCount] = { 0.0f, 0.0f, 0.0f, 0.0f };
};
