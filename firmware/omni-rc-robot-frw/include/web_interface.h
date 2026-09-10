#pragma once

/**
 * @file web_interface.h
 *
 * @brief Wi-Fi, serveur HTTP, API et telemetrie temps reel.
 *
 * Entrees  : requetes HTTP et WebSocket des clients, telemetrie du RobotController.
 * Sorties  : pages Web, reponses d'API, diffusion de la telemetrie et des logs.
 * Dialogue avec : RobotController (lecture d'etat et demandes) et ConfigManager.
 *
 * L'interface est concue pour fonctionner SANS acces Internet : toutes les
 * ressources HTML, CSS et JavaScript sont servies depuis la memoire flash de
 * l'ESP32 (systeme de fichiers LittleFS, dossier data/ du projet).
 *
 * Le serveur est asynchrone et s'execute sur le coeur 0, tandis que la boucle
 * de controle moteur tourne sur le coeur 1 : aucune requete Web ne peut
 * retarder le pilotage des moteurs.
 *
 * Cote securite, cette interface ne peut jamais contourner le failsafe :
 * elle ne fait que deposer des demandes que SafetyManager reste libre de refuser.
 */

#include <Arduino.h>

#include "config_manager.h"
#include "robot_controller.h"

/**
 * @brief Serveur Web embarque du robot.
 */
class WebInterface
{
public:
    /**
     * @brief Demarre le Wi-Fi, le systeme de fichiers et le serveur HTTP.
     *
     * En mode Station, si la connexion echoue dans le delai configure, le robot
     * bascule automatiquement en point d'acces : l'interface reste ainsi
     * toujours accessible, meme apres une mauvaise configuration reseau.
     *
     * @param configManager Configuration du robot, consultee et modifiee par l'API.
     * @param robotController Controleur interroge pour la telemetrie.
     */
    void begin(ConfigManager& configManager, RobotController& robotController);

    /**
     * @brief Diffuse la telemetrie et les logs aux clients connectes.
     *
     * A appeler regulierement depuis la boucle Arduino. La diffusion est
     * limitee en interne a la cadence d'affichage utile.
     */
    void update();

    /// Adresse IP courante du robot, sous forme texte.
    String ipAddress() const;

    /// SSID reellement utilise.
    String activeSsid() const { return currentSsid; }

    /// true si le robot est connecte a un reseau existant (mode Station).
    bool isStationMode() const { return isConnectedAsStation; }

    /// Nombre de clients WebSocket connectes.
    uint32_t connectedClientCount() const;

private:
    /// Etablit la connexion Wi-Fi, avec repli en point d'acces.
    void startWifi();

    /// Demarre le point d'acces de secours.
    void startAccessPoint(const char* reasonForFallback);

    /// Monte LittleFS et signale clairement un systeme de fichiers absent.
    bool mountFileSystem();

    /// Enregistre les routes de l'API et les fichiers statiques.
    void registerRoutes();

    /// Construit le document JSON de telemetrie diffuse aux clients.
    void buildTelemetryJson(JsonObject destination) const;

    /// Construit le document JSON de diagnostic systeme.
    void buildSystemJson(JsonObject destination) const;

    /// Diffuse les nouvelles lignes de journal aux clients WebSocket.
    void broadcastNewLogEntries();

    ConfigManager*   configManager   = nullptr;
    RobotController* robotController = nullptr;

    String   currentSsid;
    bool     isConnectedAsStation = false;
    bool     isFileSystemMounted  = false;
    uint32_t lastTelemetryPushMs  = 0;
    uint32_t lastSentLogSequence  = 0;
};
