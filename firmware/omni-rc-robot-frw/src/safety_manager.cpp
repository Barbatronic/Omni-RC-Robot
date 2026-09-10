/**
 * @file safety_manager.cpp
 *
 * @brief Implementation de la machine d'etats de securite.
 *
 * Chaque transition est journalisee avec sa cause, de facon a pouvoir
 * reconstituer apres coup pourquoi le robot s'est desarme ou coupe.
 */

#include "safety_manager.h"

#include "logger.h"

namespace
{
    constexpr const char* logModule = "SAFETY";
}

const char* robotStateName(RobotState state)
{
    switch (state) {
        case RobotState::Boot:            return "BOOT";
        case RobotState::WaitingForRadio: return "WAITING_FOR_RADIO";
        case RobotState::Disarmed:        return "DISARMED";
        case RobotState::Armed:           return "ARMED";
        case RobotState::Failsafe:        return "FAILSAFE";
        case RobotState::MotorTest:       return "MOTOR_TEST";
        case RobotState::Error:           return "ERROR";
    }
    return "UNKNOWN";
}

void SafetyManager::begin(const SafetyConfiguration& safety)
{
    configuration    = safety;
    currentState     = RobotState::Boot;
    stateEntryTimeMs = millis();
    hasSeenRadio     = false;
}

void SafetyManager::setConfiguration(const SafetyConfiguration& safety)
{
    configuration = safety;
}

void SafetyManager::transitionTo(RobotState newState, const char* reason)
{
    if (newState == currentState) {
        return;
    }

    const LogLevel level = (newState == RobotState::Failsafe || newState == RobotState::Error)
                               ? LogLevel::Error
                               : LogLevel::Info;

    Logger::log(level, logModule, "%s -> %s : %s",
                robotStateName(currentState), robotStateName(newState), reason);

    if (newState == RobotState::Failsafe) {
        ++failsafeCounter;
        // Le retour du signal ne doit jamais relancer les moteurs tout seul.
        requiresArmSwitchRelease = true;
    }

    currentState         = newState;
    stateEntryTimeMs     = millis();
    armingConditionsHold = false;
}

void SafetyManager::requestDisarm(const char* reason)
{
    if (currentState == RobotState::Armed || currentState == RobotState::MotorTest) {
        transitionTo(RobotState::Disarmed, reason);
    }

    // Pose le verrou meme si le robot etait deja desarme : un clic sur le bouton
    // d'arret doit toujours empecher un reamement tant que l'interrupteur de la
    // radiocommande n'a pas ete repasse au repos.
    requiresArmSwitchRelease = true;
}

uint32_t SafetyManager::armingHoldRemainingMs() const
{
    if (!armingConditionsHold) {
        return configuration.armingHoldMs;
    }
    const uint32_t elapsedMs = millis() - armingConditionsSinceMs;
    return (elapsedMs >= configuration.armingHoldMs) ? 0 : (configuration.armingHoldMs - elapsedMs);
}

bool SafetyManager::canEnterMotorTestMode(const SafetyInputs& inputs) const
{
    // Le test moteur n'est autorise que robot desarme, sans failsafe en cours,
    // et avec l'interrupteur d'armement de la radio en position desarmee : la
    // radiocommande ne doit jamais pouvoir commander en meme temps que le Web.
    return currentState == RobotState::Disarmed
        && !inputs.isRadioFailsafeActive
        && !inputs.isArmSwitchEngaged;
}

RobotState SafetyManager::update(const SafetyInputs& inputs)
{
    if (inputs.isRadioConnected) {
        hasSeenRadio = true;
    }

    // Le verrou d'armement se leve des que l'interrupteur est vu au repos.
    if (!inputs.isArmSwitchEngaged) {
        requiresArmSwitchRelease = false;
    }

    // --- Priorite 1 : failsafe. Aucune autre regle ne peut le contourner. -----
    const bool isRadioLost = !inputs.isRadioConnected || inputs.isRadioFailsafeActive;

    if (isRadioLost && hasSeenRadio) {
        if (currentState != RobotState::Failsafe) {
            transitionTo(RobotState::Failsafe,
                         inputs.isRadioFailsafeActive
                             ? "le recepteur signale un failsafe, moteurs coupes"
                             : "aucune trame radio valide dans le delai imparti, moteurs coupes");
        }
        return currentState;
    }

    switch (currentState) {
        case RobotState::Boot:
            transitionTo(RobotState::WaitingForRadio, "initialisation terminee");
            break;

        case RobotState::WaitingForRadio:
            if (inputs.isRadioConnected) {
                transitionTo(RobotState::Disarmed, "liaison radio etablie");
            }
            break;

        case RobotState::Failsafe:
            // On ne revient jamais directement en ARMED : la radio doit d'abord
            // etre retrouvee ET l'interrupteur d'armement repasse au repos, ce
            // qui interdit tout redemarrage moteur intempestif.
            if (inputs.isRadioConnected && !inputs.isArmSwitchEngaged) {
                transitionTo(RobotState::Disarmed,
                             "liaison radio retablie et interrupteur d'armement au repos");
            }
            break;

        case RobotState::Disarmed:
            if (inputs.isMotorTestRequested && canEnterMotorTestMode(inputs)) {
                transitionTo(RobotState::MotorTest, "test moteur demande depuis l'interface Web");
                break;
            }

            // Armement : la voie d'armement doit etre active, les manches au
            // neutre, aucun verrou en attente, et ces conditions doivent tenir
            // pendant armingHoldMs.
            if (inputs.isArmSwitchEngaged && inputs.areSticksNeutral
                && !requiresArmSwitchRelease) {
                if (!armingConditionsHold) {
                    armingConditionsHold    = true;
                    armingConditionsSinceMs = millis();
                } else if ((millis() - armingConditionsSinceMs) >= configuration.armingHoldMs) {
                    transitionTo(RobotState::Armed, "conditions d'armement maintenues");
                }
            } else {
                armingConditionsHold = false;
            }
            break;

        case RobotState::Armed:
            if (!inputs.isArmSwitchEngaged) {
                transitionTo(RobotState::Disarmed, "interrupteur d'armement relache");
            }
            break;

        case RobotState::MotorTest:
            // Le test s'arrete des que le client Web cesse de le confirmer, ou
            // si la radiocommande demande l'armement.
            if (!inputs.isMotorTestRequested) {
                transitionTo(RobotState::Disarmed, "test moteur termine ou expire");
            } else if (inputs.isArmSwitchEngaged) {
                transitionTo(RobotState::Disarmed,
                             "armement demande a la radio : test moteur interrompu");
            }
            break;

        case RobotState::Error:
            // Un defaut bloquant ne se quitte que par un redemarrage.
            break;
    }

    return currentState;
}
