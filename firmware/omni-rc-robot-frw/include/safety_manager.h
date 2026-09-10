#pragma once

/**
 * @file safety_manager.h
 *
 * @brief Machine d'etats et regles de securite du robot.
 *
 * Entrees  : etat de la liaison radio, position des manches, voie d'armement,
 *            demandes provenant de l'interface Web.
 * Sorties  : etat courant du robot et autorisation de commander les moteurs.
 * Dialogue avec : RobotController, qui lui fournit les entrees et applique ses
 *                 decisions.
 *
 * Toutes les regles de securite sont regroupees ici et nulle part ailleurs,
 * afin de rester verifiables d'un seul coup d'oeil.
 *
 * Priorite imperative des commandes, du plus fort au plus faible :
 *
 *     FAILSAFE  >  DESARMEMENT  >  TEST MOTEUR  >  COMMANDE SBUS
 *
 * Aucune fonction de l'interface Web ne peut contourner un failsafe.
 */

#include <Arduino.h>

#include "robot_config.h"

/**
 * @brief Etats du robot.
 */
enum class RobotState : uint8_t
{
    Boot,             ///< Initialisation en cours
    WaitingForRadio,  ///< Aucune trame radio valide recue depuis le demarrage
    Disarmed,         ///< Radio presente, moteurs interdits
    Armed,            ///< Pilotage RC autorise
    Failsafe,         ///< Liaison radio perdue : moteurs coupes
    MotorTest,        ///< Test moteur depuis l'interface Web, robot desarme
    Error,            ///< Defaut bloquant
};

/// Nom lisible d'un etat, utilise dans les logs et l'interface Web.
const char* robotStateName(RobotState state);

/**
 * @brief Entrees necessaires a la decision de securite, a chaque cycle.
 */
struct SafetyInputs
{
    bool isRadioConnected      = false;  ///< Trame valide recente
    bool isRadioFailsafeActive = false;  ///< Le recepteur signale un failsafe
    bool isArmSwitchEngaged    = false;  ///< Voie d'armement en position armee
    bool areSticksNeutral      = false;  ///< vx, vy et omega proches de zero
    bool isMotorTestRequested  = false;  ///< Test moteur demande et encore valide
};

/**
 * @brief Applique les regles d'armement, de desarmement et de failsafe.
 */
class SafetyManager
{
public:
    /// Initialise la machine d'etats a l'etat Boot.
    void begin(const SafetyConfiguration& safety);

    /// Met a jour les parametres de securite sans changer l'etat courant.
    void setConfiguration(const SafetyConfiguration& safety);

    /**
     * @brief Calcule le nouvel etat du robot.
     *
     * @param inputs Entrees mesurees pendant ce cycle.
     * @return Etat resultant.
     */
    RobotState update(const SafetyInputs& inputs);

    /// Force le desarmement immediat, par exemple sur demande de l'interface Web.
    ///
    /// Le reamement exigera ensuite que l'interrupteur de la radiocommande soit
    /// repasse au repos : sans cela, un interrupteur laisse en position armee
    /// ferait redemarrer le robot des la temporisation ecoulee, et le bouton
    /// d'arret de l'interface ne servirait a rien.
    void requestDisarm(const char* reason);

    RobotState state() const { return currentState; }

    /// true si le pilotage par la radiocommande est autorise.
    bool isRobotArmed() const { return currentState == RobotState::Armed; }

    /// true si le test moteur Web est en cours.
    bool isMotorTestActive() const { return currentState == RobotState::MotorTest; }

    /// true si un test moteur peut etre demarre dans l'etat courant.
    bool canEnterMotorTestMode(const SafetyInputs& inputs) const;

    /// Nombre total d'entrees en FAILSAFE depuis le demarrage.
    uint32_t failsafeEntryCount() const { return failsafeCounter; }

    /// true si l'armement attend que l'interrupteur repasse au repos.
    bool isArmSwitchCycleRequired() const { return requiresArmSwitchRelease; }

    /// Millisecondes restantes avant armement, une fois les conditions reunies.
    uint32_t armingHoldRemainingMs() const;

    /// Millisecondes ecoulees dans l'etat courant.
    uint32_t timeInCurrentStateMs() const { return millis() - stateEntryTimeMs; }

private:
    /// Change d'etat et journalise la transition.
    void transitionTo(RobotState newState, const char* reason);

    SafetyConfiguration configuration{};
    RobotState          currentState        = RobotState::Boot;
    uint32_t            stateEntryTimeMs    = 0;
    uint32_t            armingConditionsSinceMs = 0;
    bool                armingConditionsHold = false;
    uint32_t            failsafeCounter     = 0;
    bool                hasSeenRadio        = false;

    /// Verrou d'armement : impose un passage au repos de l'interrupteur.
    bool                requiresArmSwitchRelease = false;
};
