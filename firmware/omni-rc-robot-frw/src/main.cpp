/**
 * @file main.cpp
 *
 * @brief Point d'entree du firmware du robot RC omnidirectionnel.
 *
 * Ce fichier ne contient volontairement que la sequence generale du programme.
 * Toute la logique se trouve dans les modules appeles ici.
 *
 * ---------------------------------------------------------------------------
 * Repartition du travail entre les deux coeurs de l'ESP32-S3
 * ---------------------------------------------------------------------------
 *
 *   Coeur 1 - tache "control"    lecture radio, securite, mixage, moteurs
 *                                cadence fixe, priorite elevee
 *
 *   Coeur 0 - boucle Arduino     Wi-Fi, serveur Web, telemetrie, journal
 *   Coeur 0 - pile reseau        (forcee par CONFIG_ASYNC_TCP_RUNNING_CORE)
 *
 * Cette separation garantit qu'une requete Web, meme lente, ne peut jamais
 * retarder le pilotage des moteurs ni la detection d'une perte radio.
 */

#include <Arduino.h>

#include "config_manager.h"
#include "logger.h"
#include "pinout.h"
#include "robot_controller.h"
#include "web_interface.h"

namespace
{
    constexpr const char* logModule = "MAIN";

    /// Frequence de la boucle de controle moteur.
    constexpr uint32_t controlLoopFrequencyHz = 200;
    constexpr TickType_t controlLoopPeriodTicks =
        pdMS_TO_TICKS(1000 / controlLoopFrequencyHz);

    ConfigManager   configManager;
    RobotController robotController;
    WebInterface    webInterface;

    /**
     * @brief Indique si le bouton de restauration est maintenu au demarrage.
     *
     * Permet de repartir des valeurs par defaut lorsqu'une configuration
     * erronee, reseau notamment, rend l'interface Web inaccessible.
     */
    bool isConfigurationResetRequested()
    {
        if (Pinout::configResetButtonPin < 0) {
            return false;
        }

        pinMode(Pinout::configResetButtonPin, INPUT_PULLUP);
        delay(20);  // Laisse le temps a la resistance de rappel d'etablir le niveau.

        const bool isPressed =
            digitalRead(Pinout::configResetButtonPin) == (Pinout::configResetButtonIsActiveLow ? LOW : HIGH);

        if (!isPressed) {
            return false;
        }

        // Confirme un appui volontaire d'au moins une seconde.
        const uint32_t pressStartMs = millis();
        while ((millis() - pressStartMs) < 1000) {
            if (digitalRead(Pinout::configResetButtonPin) != (Pinout::configResetButtonIsActiveLow ? LOW : HIGH)) {
                return false;
            }
            delay(50);
        }
        return true;
    }

    /// Affiche le recapitulatif de demarrage sur le terminal serie.
    void logStartupDiagnostics()
    {
        const RobotConfiguration& configuration = configManager.configuration();

        LOG_INFO(logModule, "=== Omni-RC-Robot - firmware %s ===", FIRMWARE_VERSION);
        LOG_INFO(logModule, "Compile le %s %s", __DATE__, __TIME__);
        LOG_INFO(logModule, "Puce %s, %d coeur(s), %lu Ko de heap libre",
                 ESP.getChipModel(), ESP.getChipCores(),
                 static_cast<unsigned long>(ESP.getFreeHeap() / 1024));
        LOG_INFO(logModule, "Configuration version %lu",
                 static_cast<unsigned long>(configuration.version));
        LOG_INFO(logModule, "Radio sur GPIO %d, timeout %lu ms",
                 Pinout::radioRxPin,
                 static_cast<unsigned long>(configuration.radio.frameTimeoutMs));

        for (int motorIndex = 0; motorIndex < wheelCount; ++motorIndex) {
            LOG_INFO(logModule, "M%d : RPWM GPIO %d, LPWM GPIO %d, angle %.1f deg, gain %.2f%s",
                     motorIndex,
                     Pinout::motorPins[motorIndex].rightPwmPin,
                     Pinout::motorPins[motorIndex].leftPwmPin,
                     configuration.wheels[motorIndex].driveAngleDeg,
                     configuration.wheels[motorIndex].outputGain,
                     configuration.wheels[motorIndex].inverted ? " (inverse)" : "");
        }

        LOG_INFO(logModule, "PWM %lu Hz sur %u bits, puissance maximale %.0f %%",
                 static_cast<unsigned long>(configuration.drive.pwmFrequencyHz),
                 configuration.drive.pwmResolutionBits,
                 configuration.drive.maxMotorOutput * 100.0f);
    }

    /**
     * @brief Tache temps reel de pilotage, executee sur le coeur 1.
     *
     * Utilise vTaskDelayUntil pour tenir une cadence reguliere quelle que soit
     * la duree de traitement d'un cycle.
     */
    void controlTask(void* parameters)
    {
        (void)parameters;

        TickType_t lastWakeTime = xTaskGetTickCount();

        for (;;) {
            robotController.updateControl();
            vTaskDelayUntil(&lastWakeTime, controlLoopPeriodTicks);
        }
    }
}

#ifdef RADIO_PROTOCOL_SCAN
/**
 * @brief Outil de mise au point : identifie le protocole emis par le recepteur.
 *
 * Essaie les combinaisons vitesse / format / polarite des protocoles RC serie
 * courants et affiche, pour chacune, le nombre d'octets recus et un extrait
 * hexadecimal. Un protocole est reconnu a son octet d'entete caracteristique.
 *
 * Compile uniquement dans l'environnement sbus_scan, jamais dans le firmware.
 */
void runReceiverProtocolScan()
{
    struct CandidateProtocol
    {
        const char* name;
        uint32_t    baudRate;
        uint32_t    serialConfig;
        bool        isInverted;
        const char* expectedHeader;
    };

    static const CandidateProtocol candidates[] = {
        { "SBUS  (100000 8E2 inverse)",     100000, SERIAL_8E2, true,  "0F" },
        { "SBUS  (100000 8E2 non inverse)", 100000, SERIAL_8E2, false, "0F" },
        { "iBUS  (115200 8N1 non inverse)", 115200, SERIAL_8N1, false, "20 40" },
        { "iBUS  (115200 8N1 inverse)",     115200, SERIAL_8N1, true,  "20 40" },
        { "CRSF  (420000 8N1 non inverse)", 420000, SERIAL_8N1, false, "C8 / EE" },
        { "CRSF  (420000 8N1 inverse)",     420000, SERIAL_8N1, true,  "C8 / EE" },
        { "DSMX  (115200 8N1 non inverse)", 115200, SERIAL_8N1, false, "-" },
    };

    HardwareSerial scanPort(Pinout::radioUartNumber);

    LOG_INFO(logModule, "=== Identification du protocole sur GPIO %d ===", Pinout::radioRxPin);

    for (const CandidateProtocol& candidate : candidates) {
        scanPort.end();
        delay(20);
        scanPort.begin(candidate.baudRate, candidate.serialConfig,
                       Pinout::radioRxPin, -1, candidate.isInverted);

        while (scanPort.available() > 0) {
            scanPort.read();
        }

        uint8_t  capturedBytes[24] = {};
        int      capturedCount     = 0;
        uint32_t totalByteCount    = 0;

        const uint32_t listenStartMs = millis();
        while ((millis() - listenStartMs) < 400) {
            while (scanPort.available() > 0) {
                const uint8_t incomingByte = static_cast<uint8_t>(scanPort.read());
                ++totalByteCount;
                if (capturedCount < 24) {
                    capturedBytes[capturedCount++] = incomingByte;
                }
            }
            delay(1);
        }

        char hexDump[24 * 3 + 1] = {};
        for (int byteIndex = 0; byteIndex < capturedCount; ++byteIndex) {
            snprintf(hexDump + byteIndex * 3, 4, "%02X ", capturedBytes[byteIndex]);
        }

        LOG_INFO(logModule, "%-32s %5lu o/400ms  attendu %-8s  %s",
                 candidate.name,
                 static_cast<unsigned long>(totalByteCount),
                 candidate.expectedHeader,
                 capturedCount > 0 ? hexDump : "(rien)");
    }

    scanPort.end();
    LOG_INFO(logModule, "=== Fin de l'identification ===");
}
#endif

void setup()
{
    Logger::begin(115200, LogLevel::Info);

#ifdef RADIO_PROTOCOL_SCAN
    delay(500);
    runReceiverProtocolScan();
#endif

    const bool resetRequested = isConfigurationResetRequested();
    configManager.loadConfiguration(resetRequested);

    // Le controleur initialise les moteurs et les force a zero : c'est la
    // premiere chose faite sur le materiel de puissance.
    robotController.begin(configManager);

    logStartupDiagnostics();

    // La tache de pilotage demarre AVANT le reseau, et c'est important.
    //
    // L'initialisation Wi-Fi peut prendre une vingtaine de secondes lorsque le
    // reseau configure est absent : delai de connexion, balayage des reseaux
    // visibles, puis creation du point d'acces de secours. Demarrer le pilotage
    // apres rendrait le robot sourd a la radiocommande pendant tout ce temps,
    // ce qui est inacceptable en utilisation reelle, loin du reseau habituel.
    //
    // Rien dans la boucle de controle ne depend du reseau : la radio et les
    // moteurs sont deja initialises, et les moteurs sont forces a zero. La
    // tache tourne a une priorite superieure a la boucle Arduino, elle preempte
    // donc l'initialisation Wi-Fi sans en etre ralentie.
    xTaskCreatePinnedToCore(controlTask, "control", 6144, nullptr, 5, nullptr, 1);

    webInterface.begin(configManager, robotController);

    LOG_INFO(logModule,
             "Demarrage termine. Interface : http://%s - le robot reste DESARME "
             "tant que les conditions d'armement ne sont pas reunies.",
             webInterface.ipAddress().c_str());
}

void loop()
{
    webInterface.update();
    delay(5);
}
