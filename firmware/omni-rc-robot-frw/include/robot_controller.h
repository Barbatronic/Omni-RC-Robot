#pragma once

/**
 * @file robot_controller.h
 *
 * @brief Coordination generale du robot : radio, securite, mixage, moteurs.
 *
 * Entrees  : trames radio, configuration courante, demandes de l'interface Web.
 * Sorties  : commandes moteur et telemetrie temps reel.
 * Dialogue avec : tous les autres modules. C'est le seul module qui les connait
 *                 tous ; ils s'ignorent mutuellement.
 *
 * ---------------------------------------------------------------------------
 * Repartition entre les taches
 * ---------------------------------------------------------------------------
 * updateControl() est appele par une tache temps reel dediee, a frequence fixe.
 * L'interface Web s'execute dans une autre tache et ne fait qu'appeler les
 * methodes de demande ci-dessous : une requete HTTP ne peut donc jamais
 * retarder le pilotage des moteurs.
 *
 * Les echanges entre ces deux taches passent uniquement par :
 *  - telemetrySnapshot(), protege par un verrou court ;
 *  - les methodes request*(), qui deposent une demande traitee au cycle suivant.
 */

#include <Arduino.h>

#include "config_manager.h"
#include "motor_controller.h"
#include "omni_drive_mixer.h"
#include "robot_config.h"
#include "safety_manager.h"
#include "radio_receiver.h"

/**
 * @brief Photographie complete de l'etat du robot, destinee a l'interface Web.
 */
struct RobotTelemetry
{
    RobotState state = RobotState::Boot;
    RadioStatus radio{};

    uint16_t rawChannels[radioChannelCount] = {};
    uint8_t  rawRadioBytes[radioRawDumpSize] = {};  ///< Derniers octets bruts, diagnostic cablage

    DriveCommand driveCommand{};              ///< vx / vy / omega apres traitement
    float mixerOutputs[wheelCount]  = {};     ///< Sortie du mixer, avant rampe
    float motorOutputs[wheelCount]  = {};     ///< Consigne reellement appliquee
    uint32_t motorDutyCycles[wheelCount] = {};///< Rapport cyclique brut envoye au driver

    bool     isSlowModeActive   = false;
    bool     isArmSwitchEngaged = false;
    float    controlLoopRateHz  = 0.0f;
    uint32_t controlLoopCount   = 0;
    int      motorTestIndex     = -1;         ///< Moteur teste, -1 si aucun
    uint32_t failsafeCount      = 0;          ///< Entrees en FAILSAFE depuis le demarrage

    // --- Conditions d'armement, pour expliquer a l'utilisateur ce qui bloque ---
    bool     armingRadioOk            = false;  ///< Liaison radio valide, sans failsafe
    bool     armingSwitchOk           = false;  ///< Interrupteur d'armement en position armee
    bool     armingSticksOk           = false;  ///< Manches suffisamment proches du neutre
    bool     armingSwitchCycleNeeded  = false;  ///< Interrupteur a repasser au repos
    uint32_t armingHoldRemainingMs    = 0;      ///< Temps restant avant armement
};

/**
 * @brief Orchestre la boucle temps reel du robot.
 */
class RobotController
{
public:
    /**
     * @brief Initialise les sous-systemes et force les moteurs a zero.
     *
     * @param configManager Gestionnaire de configuration deja charge.
     */
    void begin(ConfigManager& configManager);

    /**
     * @brief Execute un cycle complet de controle.
     *
     * Enchaine : lecture radio, securite, calcul des consignes, mixage,
     * application aux moteurs. A appeler depuis la tache temps reel.
     */
    void updateControl();

    /// Signale que la configuration a change et doit etre rechargee au prochain cycle.
    void requestConfigurationReload();

    /**
     * @brief Demande un test moteur depuis l'interface Web.
     *
     * La demande expire d'elle-meme apres motorTestTimeoutMs : l'interface doit
     * la renouveler tant que le bouton reste enfonce. Le test s'arrete donc
     * aussi bien a la fin du geste qu'a la perte de la connexion Web.
     *
     * @param motorIndex Moteur a tester, de 0 a 3.
     * @param requestedOutput Consigne demandee, bornee par motorTestMaxOutput.
     * @param errorMessage Renseigne si la demande est refusee.
     * @return true si la demande a ete acceptee.
     */
    bool requestMotorTest(int motorIndex, float requestedOutput, String& errorMessage);

    /// Annule immediatement tout test moteur en cours.
    void cancelMotorTest();

    /// Desarme immediatement le robot depuis l'interface Web.
    void requestDisarm();

    /// Copie coherente de l'etat courant, sure depuis n'importe quelle tache.
    RobotTelemetry telemetrySnapshot() const;

    /// Etat courant du robot.
    RobotState state() const { return safetyManager.state(); }

private:
    /// Recharge la configuration si une demande est en attente.
    void applyPendingConfiguration();

    /**
     * @brief Reconnait le protocole du recepteur et adapte la calibration.
     *
     * Si le protocole detecte differe de celui enregistre, les plages de
     * calibration des axes sont remises aux valeurs typiques du nouveau
     * protocole : une calibration SBUS n'a aucun sens en iBUS et inversement.
     */
    void detectRadioProtocol();

    /// Construit vx / vy / omega a partir des voies radio et de la configuration.
    DriveCommand readDriveCommandFromRadio() const;

    /// true si la voie d'armement est en position armee.
    bool isArmSwitchEngaged() const;

    /// true si le selecteur de vitesse demande le mode lent.
    bool isSlowModeSelected() const;

    /// Calcule les consignes moteur du mode test.
    void computeMotorTestOutputs(float (&targets)[wheelCount]) const;

    /// Met a jour la LED de statut selon l'etat du robot.
    void updateStatusLed();

    /// Publie la telemetrie de ce cycle.
    void publishTelemetry(const DriveCommand& command,
                          const WheelOutputs& mixerOutputs,
                          const SafetyInputs& safetyInputs);

    ConfigManager*     configManager = nullptr;
    RobotConfiguration activeConfiguration{};

    RadioReceiver   radioReceiver;
    MotorController motorController;
    SafetyManager   safetyManager;

    // --- Etat de la tache temps reel ---------------------------------------
    uint32_t lastControlCycleMicroseconds = 0;
    uint32_t controlLoopCount             = 0;
    uint32_t loopRateWindowStartMs        = 0;
    uint32_t loopRateWindowCount          = 0;
    float    measuredLoopRateHz           = 0.0f;

    // --- Demandes venues de la tache reseau --------------------------------
    volatile bool     configurationReloadRequested = false;
    volatile int      motorTestIndex               = -1;
    volatile float    motorTestOutput              = 0.0f;
    volatile uint32_t motorTestDeadlineMs          = 0;
    volatile bool     webDisarmRequested           = false;

    RobotTelemetry telemetry{};
};
