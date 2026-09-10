/**
 * @file motor_controller.cpp
 *
 * @brief Implementation du pilotage des BTS7960 et de la rampe d'acceleration.
 */

#include "motor_controller.h"

#include "logger.h"

#include <cmath>

namespace
{
    constexpr const char* logModule = "MOTOR";

    /// Resolution minimale acceptable : en dessous, la commande moteur devient trop grossiere.
    constexpr uint8_t minimumPwmResolutionBits = 8;

    /**
     * @brief Determine la plus grande resolution PWM que le materiel accepte.
     *
     * Le LEDC impose frequence x 2^resolution <= frequence de son horloge source.
     * Cette horloge depend de la puce et de la version du core : sur l'ESP32-S3
     * elle se cale sur 40 MHz et non sur les 80 MHz du bus APB, ce qui rend par
     * exemple 20 kHz en 11 bits impossible.
     *
     * Plutot que de recalculer cette limite theorique, la resolution est
     * eprouvee sur le materiel : ledcSetup() retourne 0 en cas d'echec. On
     * descend d'un bit jusqu'a obtenir une configuration reellement acceptee.
     * Sans cela, les huit canaux echouent silencieusement et aucun moteur ne repond.
     *
     * @param probeChannel Canal LEDC utilise pour l'essai.
     * @param pwmFrequencyHz Frequence PWM demandee.
     * @param requestedBits Resolution souhaitee.
     * @return Resolution utilisable, ou 0 si aucune ne convient.
     */
    uint8_t resolveUsablePwmResolution(uint8_t probeChannel,
                                       uint32_t pwmFrequencyHz,
                                       uint8_t requestedBits)
    {
        for (uint8_t candidateBits = requestedBits;
             candidateBits >= minimumPwmResolutionBits;
             --candidateBits) {

            if (ledcSetup(probeChannel, pwmFrequencyHz, candidateBits) != 0) {
                if (candidateBits != requestedBits) {
                    LOG_WARN(logModule,
                             "Resolution PWM %u bits refusee a %lu Hz : repli sur %u bits",
                             requestedBits, static_cast<unsigned long>(pwmFrequencyHz),
                             candidateBits);
                }
                return candidateBits;
            }
        }

        LOG_ERROR(logModule,
                  "Aucune resolution PWM utilisable a %lu Hz : les moteurs ne repondront pas. "
                  "Reduire la frequence PWM dans l'onglet Conduite (20000 Hz maximum conseille).",
                  static_cast<unsigned long>(pwmFrequencyHz));
        return 0;
    }
}

// -----------------------------------------------------------------------------
// Motor
// -----------------------------------------------------------------------------

bool Motor::begin(const MotorPins& motorPins,
                  uint8_t rightChannel,
                  uint8_t leftChannel,
                  uint32_t pwmFrequencyHz,
                  uint8_t pwmResolutionBits)
{
    pins            = motorPins;
    rightPwmChannel = rightChannel;
    leftPwmChannel  = leftChannel;
    // La plage du registre LEDC va de 0 a 2^bits INCLUS : 2^bits donne un etat
    // haut permanent. S'arreter a 2^bits - 1 laisserait une micro-impulsion basse
    // a chaque periode, donc une commutation residuelle a pleine commande, la ou
    // le couple demande est maximal.
    maximumDutyCycle = (1UL << pwmResolutionBits);

    const bool isRightChannelReady =
        ledcSetup(rightPwmChannel, pwmFrequencyHz, pwmResolutionBits) != 0;
    const bool isLeftChannelReady =
        ledcSetup(leftPwmChannel, pwmFrequencyHz, pwmResolutionBits) != 0;

    ledcAttachPin(pins.rightPwmPin, rightPwmChannel);
    ledcAttachPin(pins.leftPwmPin,  leftPwmChannel);

    // Les sorties sont mises a zero avant toute activation des drivers.
    stop();

    // Les broches Enable ne sont pilotees que si elles sont reellement cablees
    // a l'ESP32. Sinon elles sont maintenues actives par le montage.
    if (pins.rightEnablePin >= 0) {
        pinMode(pins.rightEnablePin, OUTPUT);
        digitalWrite(pins.rightEnablePin, HIGH);
    }
    if (pins.leftEnablePin >= 0) {
        pinMode(pins.leftEnablePin, OUTPUT);
        digitalWrite(pins.leftEnablePin, HIGH);
    }

    return isRightChannelReady && isLeftChannelReady;
}

void Motor::setOutput(float output)
{
    if (isnan(output)) {
        output = 0.0f;
    }
    output = constrain(output, -1.0f, 1.0f);

    lastAppliedOutput = output;

    const uint32_t dutyCycle =
        static_cast<uint32_t>(fabsf(output) * static_cast<float>(maximumDutyCycle) + 0.5f);
    lastAppliedDutyCycle = dutyCycle;

    // Une seule des deux entrees recoit du PWM : l'autre est forcee a zero
    // avant, pour ne jamais faire conduire les deux demi-ponts simultanement.
    if (output > 0.0f) {
        ledcWrite(leftPwmChannel, 0);
        ledcWrite(rightPwmChannel, dutyCycle);
    } else if (output < 0.0f) {
        ledcWrite(rightPwmChannel, 0);
        ledcWrite(leftPwmChannel, dutyCycle);
    } else {
        ledcWrite(rightPwmChannel, 0);
        ledcWrite(leftPwmChannel, 0);
    }
}

void Motor::stop()
{
    lastAppliedOutput    = 0.0f;
    lastAppliedDutyCycle = 0;
    ledcWrite(rightPwmChannel, 0);
    ledcWrite(leftPwmChannel, 0);
}

// -----------------------------------------------------------------------------
// MotorController
// -----------------------------------------------------------------------------

void MotorController::begin(const DriveConfiguration& drive)
{
    driveConfiguration = drive;
    configurePwmChannels(drive);
}

void MotorController::configurePwmChannels(const DriveConfiguration& drive)
{
    const uint8_t resolutionBits =
        resolveUsablePwmResolution(0, drive.pwmFrequencyHz, drive.pwmResolutionBits);
    if (resolutionBits == 0) {
        return;
    }

    bool areAllChannelsReady = true;

    for (int motorIndex = 0; motorIndex < wheelCount; ++motorIndex) {
        // Deux canaux LEDC consecutifs par moteur : 0/1, 2/3, 4/5, 6/7.
        const uint8_t rightChannel = static_cast<uint8_t>(motorIndex * 2);
        const uint8_t leftChannel  = static_cast<uint8_t>(motorIndex * 2 + 1);

        if (!motors[motorIndex].begin(Pinout::motorPins[motorIndex],
                                      rightChannel,
                                      leftChannel,
                                      drive.pwmFrequencyHz,
                                      resolutionBits)) {
            areAllChannelsReady = false;
            LOG_ERROR(logModule, "M%d : configuration PWM refusee, ce moteur ne repondra pas",
                      motorIndex);
        }
        currentOutputs[motorIndex] = 0.0f;
    }

    if (areAllChannelsReady) {
        LOG_INFO(logModule, "4 moteurs initialises : PWM %lu Hz, %u bits, sorties a zero",
                 static_cast<unsigned long>(drive.pwmFrequencyHz), resolutionBits);
    }
}

void MotorController::setDriveConfiguration(const DriveConfiguration& drive)
{
    const bool hasPwmChanged =
        (drive.pwmFrequencyHz != driveConfiguration.pwmFrequencyHz) ||
        (drive.pwmResolutionBits != driveConfiguration.pwmResolutionBits);

    driveConfiguration = drive;

    if (!hasPwmChanged) {
        return;
    }

    // Les moteurs sont coupes avant de toucher au peripherique PWM : changer la
    // frequence d'un canal en cours de rotation produirait une impulsion non
    // maitrisee sur le pont en H.
    stopAllMotors();
    configurePwmChannels(drive);

    LOG_INFO(logModule, "PWM reconfigure a chaud : %lu Hz",
             static_cast<unsigned long>(drive.pwmFrequencyHz));
}

float MotorController::applyRateLimit(float current, float target, float deltaTimeSeconds) const
{
    // Une variation qui rapproche la sortie de zero est un freinage, les autres
    // sont des accelerations. Les deux pentes sont reglables separement.
    const bool isDecelerating = fabsf(target) < fabsf(current);
    const float ratePerSecond = isDecelerating ? driveConfiguration.maxDecelerationPerSecond
                                               : driveConfiguration.maxAccelerationPerSecond;

    if (ratePerSecond <= 0.0f) {
        return target;
    }

    const float maximumStep = ratePerSecond * deltaTimeSeconds;
    const float requestedStep = target - current;

    if (requestedStep > maximumStep) {
        return current + maximumStep;
    }
    if (requestedStep < -maximumStep) {
        return current - maximumStep;
    }
    return target;
}

void MotorController::applyOutputs(const float (&targets)[wheelCount], float deltaTimeSeconds)
{
    // Un pas de temps aberrant (premiere iteration, reveil de tache) ne doit pas
    // autoriser un saut de consigne arbitrairement grand.
    deltaTimeSeconds = constrain(deltaTimeSeconds, 0.0f, 0.1f);

    for (int motorIndex = 0; motorIndex < wheelCount; ++motorIndex) {
        currentOutputs[motorIndex] =
            applyRateLimit(currentOutputs[motorIndex], targets[motorIndex], deltaTimeSeconds);
        motors[motorIndex].setOutput(currentOutputs[motorIndex]);
    }
}

void MotorController::stopAllMotors()
{
    for (int motorIndex = 0; motorIndex < wheelCount; ++motorIndex) {
        currentOutputs[motorIndex] = 0.0f;
        motors[motorIndex].stop();
    }
}

float MotorController::appliedOutput(int motorIndex) const
{
    if (motorIndex < 0 || motorIndex >= wheelCount) {
        return 0.0f;
    }
    return motors[motorIndex].appliedOutput();
}

uint32_t MotorController::appliedDutyCycle(int motorIndex) const
{
    if (motorIndex < 0 || motorIndex >= wheelCount) {
        return 0;
    }
    return motors[motorIndex].appliedDutyCycle();
}
