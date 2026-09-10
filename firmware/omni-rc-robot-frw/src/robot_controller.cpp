/**
 * @file robot_controller.cpp
 *
 * @brief Implementation de la boucle de controle du robot.
 *
 * Deroulement d'un cycle, dans l'ordre :
 *
 *   1. prise en compte d'une eventuelle nouvelle configuration
 *   2. lecture des trames radio
 *   3. conversion des voies en vx / vy / omega
 *   4. evaluation des regles de securite
 *   5. selon l'etat : mixage et pilotage, test moteur, ou arret
 *   6. publication de la telemetrie et mise a jour de la LED
 */

#include "robot_controller.h"

#include "logger.h"
#include "pinout.h"

#include <cmath>
#include <cstring>

namespace
{
    constexpr const char* logModule = "ROBOT";

    /// Protege la copie de telemetrie partagee entre la tache de controle et le Web.
    portMUX_TYPE telemetryLock = portMUX_INITIALIZER_UNLOCKED;

    /// Periode de mesure de la frequence de boucle.
    constexpr uint32_t loopRateWindowMs = 1000;

    // -------------------------------------------------------------------------
    // LED de statut
    //
    // Codes lumineux :
    //   double flash        attente de la radio
    //   clignotement lent   desarme
    //   fixe                arme
    //   clignotement rapide failsafe
    //   pulsation           test moteur
    // -------------------------------------------------------------------------

    void writeStatusLed(bool isLit, uint8_t red, uint8_t green, uint8_t blue)
    {
        if (Pinout::statusLedPin < 0) {
            return;
        }

        if (Pinout::statusLedIsAddressable) {
#ifdef RGB_BUILTIN
            neopixelWrite(Pinout::statusLedPin,
                          isLit ? red : 0, isLit ? green : 0, isLit ? blue : 0);
#else
            (void)red; (void)green; (void)blue;
#endif
        } else {
            const bool pinLevel = Pinout::statusLedIsActiveLow ? !isLit : isLit;
            digitalWrite(Pinout::statusLedPin, pinLevel ? HIGH : LOW);
        }
    }

    /// true pendant la phase allumee d'un motif clignotant de periode @p periodMs.
    bool isBlinkPhaseOn(uint32_t periodMs, uint32_t onDurationMs)
    {
        return (millis() % periodMs) < onDurationMs;
    }
}

void RobotController::begin(ConfigManager& manager)
{
    configManager = &manager;
    manager.copyConfigurationTo(activeConfiguration, portMAX_DELAY);

    if (Pinout::statusLedPin >= 0 && !Pinout::statusLedIsAddressable) {
        pinMode(Pinout::statusLedPin, OUTPUT);
    }

    // Les moteurs sont initialises et forces a zero avant toute autre chose :
    // aucun mouvement ne doit pouvoir se produire pendant le demarrage.
    motorController.begin(activeConfiguration.drive);
    motorController.stopAllMotors();

    safetyManager.begin(activeConfiguration.safety);
    radioReceiver.begin(Pinout::radioUartNumber,
                        Pinout::radioRxPin,
                        activeConfiguration.radio.frameTimeoutMs,
                        activeConfiguration.radio.protocol);

    detectRadioProtocol();

    lastControlCycleMicroseconds = micros();
    loopRateWindowStartMs        = millis();

    LOG_INFO(logModule, "Controleur pret, moteurs a zero");
}

void RobotController::requestConfigurationReload()
{
    configurationReloadRequested = true;
}

void RobotController::applyPendingConfiguration()
{
    if (!configurationReloadRequested || configManager == nullptr) {
        return;
    }

    // La copie est faite sous le verrou de ConfigManager, sans jamais bloquer :
    // si la tache Web est en train d'ecrire, on retentera au cycle suivant.
    if (!configManager->copyConfigurationTo(activeConfiguration, 0)) {
        return;
    }
    configurationReloadRequested = false;

    motorController.setDriveConfiguration(activeConfiguration.drive);
    safetyManager.setConfiguration(activeConfiguration.safety);
    radioReceiver.setFrameTimeoutMs(activeConfiguration.radio.frameTimeoutMs);

    LOG_INFO(logModule, "Nouvelle configuration prise en compte");
}

void RobotController::detectRadioProtocol()
{
    const RadioProtocol configuredProtocol = activeConfiguration.radio.protocol;

    if (!activeConfiguration.radio.autoDetectProtocol) {
        LOG_INFO(logModule, "Protocole radio impose par la configuration : %s",
                 radioProtocolName(configuredProtocol));
        return;
    }

    // Emetteur eteint, aucun protocole ne produira de trame : ce n'est pas une
    // erreur au demarrage, le reglage enregistre est simplement conserve.
    if (!radioReceiver.detectProtocol(configuredProtocol, 400)) {
        LOG_WARN(logModule,
                 "Aucune trame radio au demarrage. Verifier que l'emetteur est allume, "
                 "que le recepteur est sur sa sortie serie (SBUS ou iBUS) et le cablage "
                 "de GPIO %d. Le protocole sera reconnu automatiquement des la premiere trame.",
                 Pinout::radioRxPin);
        return;
    }

    const RadioProtocol detectedProtocol = radioReceiver.activeProtocol();
    if (detectedProtocol == configuredProtocol) {
        return;
    }

    // Les echelles de valeurs brutes different d'un protocole a l'autre : la
    // calibration enregistree doit etre remplacee, sinon les manches seraient
    // interpretes n'importe comment.
    LOG_WARN(logModule,
             "Protocole detecte (%s) different de celui enregistre (%s) : "
             "calibration des axes remise aux valeurs par defaut du %s.",
             radioProtocolName(detectedProtocol),
             radioProtocolName(configuredProtocol),
             radioProtocolName(detectedProtocol));

    RobotConfiguration updatedConfiguration = activeConfiguration;
    applyProtocolCalibrationDefaults(updatedConfiguration, detectedProtocol);
    configManager->replaceConfiguration(updatedConfiguration);
    configManager->copyConfigurationTo(activeConfiguration, portMAX_DELAY);
}

DriveCommand RobotController::readDriveCommandFromRadio() const
{
    const RadioConfiguration& radio = activeConfiguration.radio;

    DriveCommand command;

    // Sans liaison radio valide, les voies conservent leur derniere valeur ou
    // valent zero, ce qui se normaliserait en butee et afficherait des consignes
    // inventees. On retourne explicitement une consigne nulle : les moteurs sont
    // deja coupes par la machine d'etats, et la telemetrie reste honnete.
    if (!radioReceiver.status().isConnected) {
        return command;
    }
    command.translationX = normalizeRadioChannel(
        radioReceiver.rawChannel(radio.translationX.channelIndex), radio.translationX);
    command.translationY = normalizeRadioChannel(
        radioReceiver.rawChannel(radio.translationY.channelIndex), radio.translationY);
    command.rotation = normalizeRadioChannel(
        radioReceiver.rawChannel(radio.rotation.channelIndex), radio.rotation);

    // Plafonds separes pour la translation et la rotation.
    command.translationX *= activeConfiguration.drive.maxTranslationOutput;
    command.translationY *= activeConfiguration.drive.maxTranslationOutput;
    command.rotation     *= activeConfiguration.drive.maxRotationOutput;

    if (isSlowModeSelected()) {
        const float scale = activeConfiguration.drive.slowModeScale;
        command.translationX *= scale;
        command.translationY *= scale;
        command.rotation     *= scale;
    }

    return command;
}

bool RobotController::isArmSwitchEngaged() const
{
    const RadioConfiguration& radio = activeConfiguration.radio;
    const uint16_t rawValue = radioReceiver.rawChannel(radio.armChannelIndex);
    const bool isAboveThreshold = rawValue > static_cast<uint16_t>(radio.armThresholdRawValue);

    return radio.armChannelIsInverted ? !isAboveThreshold : isAboveThreshold;
}

bool RobotController::isSlowModeSelected() const
{
    const RadioConfiguration& radio = activeConfiguration.radio;
    if (radio.speedModeChannelIndex < 0) {
        return false;
    }

    const uint16_t rawValue = radioReceiver.rawChannel(radio.speedModeChannelIndex);
    return rawValue <= static_cast<uint16_t>(radio.speedModeThresholdRawValue);
}

bool RobotController::requestMotorTest(int requestedMotorIndex, float requestedOutput,
                                       String& errorMessage)
{
    if (requestedMotorIndex < 0 || requestedMotorIndex >= wheelCount) {
        errorMessage = "Index de moteur invalide : attendu entre 0 et 3";
        return false;
    }

    if (safetyManager.state() != RobotState::Disarmed
        && safetyManager.state() != RobotState::MotorTest) {
        errorMessage = "Test moteur refuse : le robot doit etre DISARMED (etat actuel ";
        errorMessage += robotStateName(safetyManager.state());
        errorMessage += ")";
        return false;
    }

    if (isArmSwitchEngaged()) {
        errorMessage = "Test moteur refuse : l'interrupteur d'armement de la radio est actif";
        return false;
    }

    const float limit = activeConfiguration.safety.motorTestMaxOutput;
    const float clampedOutput = constrain(requestedOutput, -limit, limit);

    motorTestIndex      = requestedMotorIndex;
    motorTestOutput     = clampedOutput;
    motorTestDeadlineMs = millis() + activeConfiguration.safety.motorTestTimeoutMs;

    return true;
}

void RobotController::cancelMotorTest()
{
    motorTestIndex      = -1;
    motorTestOutput     = 0.0f;
    motorTestDeadlineMs = 0;
}

void RobotController::requestDisarm()
{
    webDisarmRequested = true;
}

void RobotController::computeMotorTestOutputs(float (&targets)[wheelCount]) const
{
    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        targets[wheelIndex] = 0.0f;
    }

    const int testedIndex = motorTestIndex;
    if (testedIndex >= 0 && testedIndex < wheelCount) {
        // Le plafond est reapplique ici : meme si la configuration a change
        // depuis l'acceptation de la demande, la sortie reste bridee.
        const float limit = activeConfiguration.safety.motorTestMaxOutput;
        targets[testedIndex] = constrain(motorTestOutput, -limit, limit);
    }
}

void RobotController::updateControl()
{
    applyPendingConfiguration();

    const uint32_t nowMicroseconds = micros();
    const float deltaTimeSeconds =
        static_cast<float>(nowMicroseconds - lastControlCycleMicroseconds) / 1000000.0f;
    lastControlCycleMicroseconds = nowMicroseconds;

    // --- 1. Radio ----------------------------------------------------------
    radioReceiver.readIncomingFrames();
    const RadioStatus radioStatus = radioReceiver.status();

    // --- 2. Consignes de pilotage ------------------------------------------
    const DriveCommand command = readDriveCommandFromRadio();

    // --- 3. Securite -------------------------------------------------------
    const uint32_t nowMs = millis();
    const bool isMotorTestStillRequested =
        (motorTestIndex >= 0) && (static_cast<int32_t>(motorTestDeadlineMs - nowMs) > 0);

    if (motorTestIndex >= 0 && !isMotorTestStillRequested) {
        cancelMotorTest();
    }

    const float neutralTolerance = activeConfiguration.safety.armingNeutralTolerance;

    SafetyInputs safetyInputs;
    safetyInputs.isRadioConnected      = radioStatus.isConnected;
    safetyInputs.isRadioFailsafeActive = radioStatus.isFailsafeActive;
    safetyInputs.isArmSwitchEngaged    = isArmSwitchEngaged();
    safetyInputs.areSticksNeutral      = fabsf(command.translationX) < neutralTolerance
                                      && fabsf(command.translationY) < neutralTolerance
                                      && fabsf(command.rotation)     < neutralTolerance;
    safetyInputs.isMotorTestRequested  = isMotorTestStillRequested;

    if (webDisarmRequested) {
        webDisarmRequested = false;
        cancelMotorTest();
        safetyInputs.isMotorTestRequested = false;
        safetyManager.requestDisarm("arret demande depuis l'interface Web");
    }

    const RobotState state = safetyManager.update(safetyInputs);

    // --- 4. Sorties moteur -------------------------------------------------
    // Le mixage est calcule a chaque cycle, y compris robot desarme. Le calcul
    // ne commande rien par lui-meme : seul l'etat ARMED transmet le resultat aux
    // moteurs. La telemetrie montre ainsi en permanence ce que la commande
    // radio produirait, ce qui permet de verifier toute la chaine radio -> axes
    // -> mixer -> roues sans jamais faire tourner un moteur.
    WheelOutputs mixerOutputs = computeWheelCommands(command,
                                                     activeConfiguration.wheels,
                                                     activeConfiguration.drive.maxMotorOutput,
                                                     activeConfiguration.drive.minMotorOutput,
                                                     activeConfiguration.drive.normalizeTranslation);
    float motorTargets[wheelCount] = { 0.0f, 0.0f, 0.0f, 0.0f };

    switch (state) {
        case RobotState::Armed:
            memcpy(motorTargets, mixerOutputs.values, sizeof(motorTargets));
            motorController.applyOutputs(motorTargets, deltaTimeSeconds);
            break;

        case RobotState::MotorTest:
            computeMotorTestOutputs(motorTargets);
            motorController.applyOutputs(motorTargets, deltaTimeSeconds);
            break;

        default:
            // BOOT, WAITING_FOR_RADIO, DISARMED, FAILSAFE, ERROR : arret immediat,
            // sans passer par la rampe d'acceleration.
            if (motorTestIndex >= 0) {
                cancelMotorTest();
            }
            motorController.stopAllMotors();
            break;
    }

    // --- 5. Telemetrie et LED ---------------------------------------------
    ++controlLoopCount;
    ++loopRateWindowCount;

    const uint32_t windowElapsedMs = nowMs - loopRateWindowStartMs;
    if (windowElapsedMs >= loopRateWindowMs) {
        measuredLoopRateHz = (static_cast<float>(loopRateWindowCount) * 1000.0f)
                           / static_cast<float>(windowElapsedMs);
        loopRateWindowCount   = 0;
        loopRateWindowStartMs = nowMs;
    }

    publishTelemetry(command, mixerOutputs, safetyInputs);
    updateStatusLed();
}

void RobotController::publishTelemetry(const DriveCommand& command,
                                       const WheelOutputs& mixerOutputs,
                                       const SafetyInputs& safetyInputs)
{
    RobotTelemetry snapshot;
    snapshot.state = safetyManager.state();
    snapshot.radio = radioReceiver.status();
    radioReceiver.copyRawChannels(snapshot.rawChannels);
    radioReceiver.copyRawByteDump(snapshot.rawRadioBytes);
    snapshot.driveCommand = command;

    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        snapshot.mixerOutputs[wheelIndex]    = mixerOutputs.values[wheelIndex];
        snapshot.motorOutputs[wheelIndex]    = motorController.appliedOutput(wheelIndex);
        snapshot.motorDutyCycles[wheelIndex] = motorController.appliedDutyCycle(wheelIndex);
    }

    snapshot.isSlowModeActive   = isSlowModeSelected();
    snapshot.isArmSwitchEngaged = isArmSwitchEngaged();
    snapshot.controlLoopRateHz  = measuredLoopRateHz;
    snapshot.controlLoopCount   = controlLoopCount;
    snapshot.motorTestIndex     = motorTestIndex;
    snapshot.failsafeCount      = safetyManager.failsafeEntryCount();

    snapshot.armingRadioOk           = safetyInputs.isRadioConnected
                                    && !safetyInputs.isRadioFailsafeActive;
    snapshot.armingSwitchOk          = safetyInputs.isArmSwitchEngaged;
    snapshot.armingSticksOk          = safetyInputs.areSticksNeutral;
    snapshot.armingSwitchCycleNeeded = safetyManager.isArmSwitchCycleRequired();
    snapshot.armingHoldRemainingMs   = safetyManager.armingHoldRemainingMs();

    taskENTER_CRITICAL(&telemetryLock);
    telemetry = snapshot;
    taskEXIT_CRITICAL(&telemetryLock);
}

RobotTelemetry RobotController::telemetrySnapshot() const
{
    RobotTelemetry snapshot;

    taskENTER_CRITICAL(&telemetryLock);
    snapshot = telemetry;
    taskEXIT_CRITICAL(&telemetryLock);

    return snapshot;
}

void RobotController::updateStatusLed()
{
    switch (safetyManager.state()) {
        case RobotState::Armed:
            writeStatusLed(true, 0, 40, 0);                              // vert fixe
            break;

        case RobotState::Failsafe:
            writeStatusLed(isBlinkPhaseOn(200, 100), 60, 0, 0);           // rouge rapide
            break;

        case RobotState::MotorTest:
            writeStatusLed(isBlinkPhaseOn(400, 200), 40, 20, 0);          // orange
            break;

        case RobotState::Disarmed:
            writeStatusLed(isBlinkPhaseOn(1500, 200), 0, 0, 40);          // bleu lent
            break;

        case RobotState::Error:
            writeStatusLed(true, 60, 0, 0);                               // rouge fixe
            break;

        case RobotState::Boot:
        case RobotState::WaitingForRadio:
        default: {
            // Double flash : deux impulsions courtes puis une pause.
            const uint32_t phaseMs = millis() % 1200;
            const bool isLit = (phaseMs < 100) || (phaseMs >= 200 && phaseMs < 300);
            writeStatusLed(isLit, 40, 25, 0);                             // jaune
            break;
        }
    }
}
