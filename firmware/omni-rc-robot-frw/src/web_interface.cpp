/**
 * @file web_interface.cpp
 *
 * @brief Implementation du Wi-Fi, du serveur HTTP et de l'API du robot.
 */

#include "web_interface.h"

#include <AsyncJson.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <esp_system.h>

#include "logger.h"
#include "omni_drive_mixer.h"
#include "pinout.h"

namespace
{
    constexpr const char* logModule = "WEB";

    /// Cadence de diffusion de la telemetrie : suffisant pour l'oeil, sans
    /// charger inutilement le reseau ni la boucle de controle.
    constexpr uint32_t telemetryIntervalMs = 100;  // 10 Hz

    /// Nom mDNS : l'interface est joignable sur http://omni-robot.local
    constexpr const char* mdnsHostName = "omni-robot";

    AsyncWebServer httpServer(80);
    AsyncWebSocket webSocket("/ws");

    RobotController* webRobotController = nullptr;

    /// Repond a une requete avec un objet JSON.
    void sendJsonResponse(AsyncWebServerRequest* request, int statusCode, JsonDocument& document)
    {
        String payload;
        serializeJson(document, payload);
        request->send(statusCode, "application/json", payload);
    }

    /// Repond avec un message d'erreur explicite.
    void sendErrorResponse(AsyncWebServerRequest* request, int statusCode, const String& message)
    {
        JsonDocument document;
        document["ok"]    = false;
        document["error"] = message;
        sendJsonResponse(request, statusCode, document);
    }

    void sendSuccessResponse(AsyncWebServerRequest* request, const String& message)
    {
        JsonDocument document;
        document["ok"]      = true;
        document["message"] = message;
        sendJsonResponse(request, 200, document);
    }

    /// Page minimale servie lorsque l'interface n'a pas ete televersee en flash.
    const char* missingFileSystemPage()
    {
        return "<!doctype html><html lang=\"fr\"><head><meta charset=\"utf-8\">"
               "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
               "<title>Omni-RC-Robot</title>"
               "<style>body{font-family:system-ui,sans-serif;margin:2rem;line-height:1.6;"
               "background:#12151b;color:#e6e8ee}code{background:#232936;padding:.15rem .4rem;"
               "border-radius:4px}</style></head><body>"
               "<h1>Interface Web absente</h1>"
               "<p>Le firmware fonctionne, mais les fichiers de l'interface ne sont pas "
               "presents dans la memoire flash.</p>"
               "<p><b>A faire :</b> televerser le contenu du dossier <code>data/</code> avec "
               "la commande PlatformIO <code>pio run -t uploadfs</code> "
               "(ou <i>Upload Filesystem Image</i> dans VSCode), puis redemarrer la carte.</p>"
               "<p>L'API reste utilisable, par exemple <code>/api/status</code>.</p>"
               "</body></html>";
    }

    /// Cause du dernier redemarrage, en clair.
    const char* resetReasonText()
    {
        static char unknownReasonText[32];

        switch (esp_reset_reason()) {
            case ESP_RST_POWERON:  return "mise sous tension";
            case ESP_RST_EXT:      return "reset externe";
            case ESP_RST_SW:       return "redemarrage logiciel";
            case ESP_RST_PANIC:    return "exception logicielle";
            case ESP_RST_INT_WDT:  return "watchdog d'interruption";
            case ESP_RST_TASK_WDT: return "watchdog de tache";
            case ESP_RST_WDT:      return "watchdog";
            case ESP_RST_BROWNOUT: return "tension d'alimentation insuffisante";
            default:
                // Les causes ajoutees par les versions recentes de l'ESP-IDF
                // (reset par USB ou JTAG) ne sont pas nommees ici : on affiche
                // le code brut plutot qu'un "inconnue" inexploitable.
                snprintf(unknownReasonText, sizeof(unknownReasonText),
                         "code %d", static_cast<int>(esp_reset_reason()));
                return unknownReasonText;
        }
    }

    /**
     * @brief Explique pourquoi la connexion au reseau a echoue.
     *
     * Balaye les reseaux visibles depuis la position du robot et indique si le
     * SSID recherche en fait partie. Cela distingue immediatement les deux
     * causes les plus frequentes : reseau hors de portee ou absent (souvent un
     * reseau 5 GHz, que l'ESP32 ne sait pas recevoir) et mot de passe errone.
     */
    void logStationFailureDiagnostics(const char* expectedSsid)
    {
        LOG_WARN(logModule, "Connexion a '%s' echouee, recherche des reseaux visibles...",
                 expectedSsid);

        // La tentative de connexion doit etre interrompue avant le balayage :
        // un scan lance pendant que la station essaie encore de s'associer
        // retourne zero reseau, ce qui donnerait un diagnostic trompeur.
        WiFi.disconnect(false, false);
        delay(150);

        const int networkCount = WiFi.scanNetworks();
        if (networkCount <= 0) {
            LOG_ERROR(logModule,
                      "Aucun reseau 2,4 GHz detecte. Verifier l'antenne de la carte "
                      "et la portee du point d'acces.");
            WiFi.scanDelete();
            return;
        }

        bool wasExpectedSsidFound = false;
        String caseInsensitiveMatch;

        for (int networkIndex = 0; networkIndex < networkCount; ++networkIndex) {
            const String foundSsid = WiFi.SSID(networkIndex);
            const bool isExpected = (foundSsid == expectedSsid);
            wasExpectedSsidFound |= isExpected;

            // Un SSID ne differant que par la casse est une erreur frequente et
            // difficile a reperer a l'oeil : les SSID sont sensibles a la casse.
            if (!isExpected && foundSsid.equalsIgnoreCase(expectedSsid)) {
                caseInsensitiveMatch = foundSsid;
            }

            LOG_INFO(logModule, "  %s%s  %d dBm  %s",
                     isExpected ? "-> " : "   ",
                     foundSsid.c_str(),
                     WiFi.RSSI(networkIndex),
                     WiFi.encryptionType(networkIndex) == WIFI_AUTH_OPEN ? "ouvert" : "protege");
        }

        if (wasExpectedSsidFound) {
            LOG_ERROR(logModule,
                      "Le reseau '%s' est bien visible : le mot de passe est probablement "
                      "errone, ou le point d'acces refuse la connexion.", expectedSsid);
        } else if (!caseInsensitiveMatch.isEmpty()) {
            LOG_ERROR(logModule,
                      "Le reseau visible s'appelle '%s' et non '%s' : seule la casse differe. "
                      "Les SSID y sont sensibles. Corriger WIFI_STATION_SSID dans "
                      "include/wifi_secrets.h.",
                      caseInsensitiveMatch.c_str(), expectedSsid);
        } else {
            LOG_ERROR(logModule,
                      "Le reseau '%s' n'apparait pas parmi les %d reseaux 2,4 GHz detectes. "
                      "Verifier le nom exact, la portee, et que le reseau n'est pas en 5 GHz "
                      "uniquement (l'ESP32-S3 ne recoit que le 2,4 GHz).",
                      expectedSsid, networkCount);
        }

        WiFi.scanDelete();
    }

    void handleWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                              AwsEventType type, void* argument, uint8_t* data, size_t length)
    {
        (void)argument;
        (void)data;
        (void)length;

        if (type == WS_EVT_CONNECT) {
            LOG_INFO(logModule, "Client Web connecte (#%lu, %s)",
                     static_cast<unsigned long>(client->id()),
                     client->remoteIP().toString().c_str());
        } else if (type == WS_EVT_DISCONNECT) {
            LOG_INFO(logModule, "Client Web deconnecte (#%lu)",
                     static_cast<unsigned long>(client->id()));

            // Plus aucun client : tout test moteur en cours est arrete, comme
            // exige par les regles de securite du mode test.
            if (server->count() == 0 && webRobotController != nullptr) {
                webRobotController->cancelMotorTest();
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Demarrage
// -----------------------------------------------------------------------------

void WebInterface::begin(ConfigManager& manager, RobotController& controller)
{
    configManager      = &manager;
    robotController    = &controller;
    webRobotController = &controller;

    isFileSystemMounted = mountFileSystem();

    startWifi();
    registerRoutes();

    webSocket.onEvent(handleWebSocketEvent);
    httpServer.addHandler(&webSocket);
    httpServer.begin();

    if (MDNS.begin(mdnsHostName)) {
        MDNS.addService("http", "tcp", 80);
        LOG_INFO(logModule, "Interface accessible sur http://%s.local", mdnsHostName);
    }

    LOG_INFO(logModule, "Serveur Web demarre sur http://%s", ipAddress().c_str());
}

bool WebInterface::mountFileSystem()
{
    if (LittleFS.begin(false)) {
        LOG_INFO(logModule, "Systeme de fichiers LittleFS monte (%lu octets utilises sur %lu)",
                 static_cast<unsigned long>(LittleFS.usedBytes()),
                 static_cast<unsigned long>(LittleFS.totalBytes()));
        return true;
    }

    LOG_ERROR(logModule,
              "LittleFS introuvable : l'interface Web n'a pas ete televersee. "
              "Lancer 'pio run -t uploadfs' pour envoyer le dossier data/");
    return false;
}

void WebInterface::startWifi()
{
    const WifiConfiguration& wifi = configManager->configuration().wifi;

    WiFi.persistent(false);

    if (wifi.mode == WifiMode::Station && wifi.stationSsid[0] != '\0') {
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);
        WiFi.begin(wifi.stationSsid, wifi.stationPassword);

        LOG_INFO(logModule, "Connexion au reseau '%s'...", wifi.stationSsid);

        const uint32_t connectionStartMs = millis();
        while (WiFi.status() != WL_CONNECTED
               && (millis() - connectionStartMs) < wifi.stationConnectTimeoutMs) {
            delay(200);
        }

        if (WiFi.status() == WL_CONNECTED) {
            isConnectedAsStation = true;
            currentSsid          = wifi.stationSsid;
            LOG_INFO(logModule, "Connecte a '%s', adresse IP %s, RSSI %d dBm",
                     currentSsid.c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
            return;
        }

        logStationFailureDiagnostics(wifi.stationSsid);
        startAccessPoint("connexion au reseau impossible dans le delai imparti");
        return;
    }

    startAccessPoint(nullptr);
}

void WebInterface::startAccessPoint(const char* reasonForFallback)
{
    const WifiConfiguration& wifi = configManager->configuration().wifi;

    if (reasonForFallback != nullptr) {
        LOG_WARN(logModule, "Repli en point d'acces : %s", reasonForFallback);
    }

    // Le SSID par defaut porte les derniers chiffres de l'identifiant de puce,
    // ce qui distingue plusieurs robots sur un meme site.
    String accessPointSsid = wifi.accessPointSsid;
    if (accessPointSsid.isEmpty()) {
        const uint32_t chipIdSuffix =
            static_cast<uint32_t>(ESP.getEfuseMac() & 0xFFFFULL);
        char generatedSsid[24];
        snprintf(generatedSsid, sizeof(generatedSsid), "RobotRC-%04X",
                 static_cast<unsigned>(chipIdSuffix));
        accessPointSsid = generatedSsid;
    }

    const bool hasPassword = strlen(wifi.accessPointPassword) >= 8;

    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    const bool wasStarted = hasPassword
                                ? WiFi.softAP(accessPointSsid.c_str(), wifi.accessPointPassword)
                                : WiFi.softAP(accessPointSsid.c_str());

    isConnectedAsStation = false;
    currentSsid          = accessPointSsid;

    if (wasStarted) {
        LOG_INFO(logModule, "Point d'acces '%s' actif (%s), adresse IP %s",
                 accessPointSsid.c_str(),
                 hasPassword ? "protege par mot de passe" : "ouvert",
                 WiFi.softAPIP().toString().c_str());
    } else {
        LOG_ERROR(logModule, "Demarrage du point d'acces impossible : interface Web indisponible");
    }
}

String WebInterface::ipAddress() const
{
    return isConnectedAsStation ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
}

uint32_t WebInterface::connectedClientCount() const
{
    return webSocket.count();
}

// -----------------------------------------------------------------------------
// Documents JSON
// -----------------------------------------------------------------------------

void WebInterface::buildTelemetryJson(JsonObject destination) const
{
    const RobotTelemetry telemetry = robotController->telemetrySnapshot();

    destination["state"] = robotStateName(telemetry.state);

    JsonObject radio = destination["radio"].to<JsonObject>();
    radio["protocol"]      = radioProtocolName(telemetry.radio.protocol);
    radio["connected"]     = telemetry.radio.isConnected;
    radio["failsafe"]      = telemetry.radio.isFailsafeActive;
    radio["frameLost"]     = telemetry.radio.isFrameLost;
    radio["frameAgeMs"]    = (telemetry.radio.frameAgeMs == UINT32_MAX)
                                ? -1 : static_cast<int32_t>(telemetry.radio.frameAgeMs);
    radio["frameRateHz"]   = telemetry.radio.frameRateHz;
    radio["validFrames"]   = telemetry.radio.validFrameCount;
    radio["frameLostCount"]= telemetry.radio.frameLostCount;
    radio["errorCount"]    = telemetry.radio.decodingErrorCount;
    radio["rawBytes"]      = telemetry.radio.rawByteCount;

    // Trace hexadecimale des derniers octets, pour diagnostiquer le cablage.
    char hexDump[radioRawDumpSize * 3 + 1] = {};
    for (int byteIndex = 0; byteIndex < radioRawDumpSize; ++byteIndex) {
        snprintf(hexDump + byteIndex * 3, 4, "%02X ", telemetry.rawRadioBytes[byteIndex]);
    }
    radio["rawDump"] = hexDump;

    JsonArray channels = destination["channels"].to<JsonArray>();
    for (int channelIndex = 0; channelIndex < radioChannelCount; ++channelIndex) {
        channels.add(telemetry.rawChannels[channelIndex]);
    }

    JsonObject input = destination["input"].to<JsonObject>();
    input["vx"]    = telemetry.driveCommand.translationX;
    input["vy"]    = telemetry.driveCommand.translationY;
    input["omega"] = telemetry.driveCommand.rotation;

    JsonArray mixerOutputs = destination["mixer"].to<JsonArray>();
    JsonArray motorOutputs = destination["motors"].to<JsonArray>();
    JsonArray dutyCycles   = destination["duty"].to<JsonArray>();
    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        mixerOutputs.add(telemetry.mixerOutputs[wheelIndex]);
        motorOutputs.add(telemetry.motorOutputs[wheelIndex]);
        dutyCycles.add(telemetry.motorDutyCycles[wheelIndex]);
    }

    destination["armSwitch"]     = telemetry.isArmSwitchEngaged;
    destination["slowMode"]      = telemetry.isSlowModeActive;
    destination["loopRateHz"]    = telemetry.controlLoopRateHz;
    destination["motorTest"]     = telemetry.motorTestIndex;
    destination["failsafeCount"] = telemetry.failsafeCount;

    // Detail des conditions d'armement : l'interface doit pouvoir dire
    // precisement ce qui empeche le robot de s'armer.
    JsonObject arming = destination["arming"].to<JsonObject>();
    arming["radioOk"]          = telemetry.armingRadioOk;
    arming["switchOk"]         = telemetry.armingSwitchOk;
    arming["sticksOk"]         = telemetry.armingSticksOk;
    arming["switchCycleNeeded"]= telemetry.armingSwitchCycleNeeded;
    arming["holdRemainingMs"]  = telemetry.armingHoldRemainingMs;
    destination["unsavedConfig"] = configManager->hasUnsavedChanges();
}

void WebInterface::buildSystemJson(JsonObject destination) const
{
    destination["firmwareVersion"] = FIRMWARE_VERSION;
    destination["buildDate"]       = __DATE__ " " __TIME__;
    destination["chipModel"]       = ESP.getChipModel();
    destination["chipCores"]       = ESP.getChipCores();
    destination["configVersion"]   = configManager->configuration().version;
    destination["uptimeSeconds"]   = millis() / 1000;
    destination["resetReason"]     = resetReasonText();
    destination["freeHeap"]        = ESP.getFreeHeap();
    destination["minimumFreeHeap"] = ESP.getMinFreeHeap();
    destination["wifiMode"]        = isConnectedAsStation ? "station" : "ap";
    destination["ssid"]            = currentSsid;
    destination["ipAddress"]       = ipAddress();
    destination["rssi"]            = isConnectedAsStation ? WiFi.RSSI() : 0;
    destination["webClients"]      = webSocket.count();
    destination["fileSystem"]      = isFileSystemMounted;

    JsonObject pins = destination["pins"].to<JsonObject>();
    pins["radioRx"]   = Pinout::radioRxPin;
    pins["statusLed"] = Pinout::statusLedPin;
    JsonArray motorPins = pins["motors"].to<JsonArray>();
    for (int motorIndex = 0; motorIndex < wheelCount; ++motorIndex) {
        JsonObject motorPinObject = motorPins.add<JsonObject>();
        motorPinObject["rpwm"] = Pinout::motorPins[motorIndex].rightPwmPin;
        motorPinObject["lpwm"] = Pinout::motorPins[motorIndex].leftPwmPin;
    }
}

// -----------------------------------------------------------------------------
// Routes
// -----------------------------------------------------------------------------

void WebInterface::registerRoutes()
{
    // --- Etat temps reel ---------------------------------------------------
    httpServer.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        JsonDocument document;
        JsonObject root = document.to<JsonObject>();
        buildTelemetryJson(root);
        sendJsonResponse(request, 200, document);
    });

    // --- Diagnostic systeme ------------------------------------------------
    httpServer.on("/api/system", HTTP_GET, [this](AsyncWebServerRequest* request) {
        JsonDocument document;
        JsonObject root = document.to<JsonObject>();
        buildSystemJson(root);
        sendJsonResponse(request, 200, document);
    });

    // --- Configuration : lecture -------------------------------------------
    // Les mots de passe Wi-Fi sont masques : ils ne transitent jamais en clair.
    httpServer.on("/api/config", HTTP_GET, [this](AsyncWebServerRequest* request) {
        JsonDocument document;
        JsonObject root = document.to<JsonObject>();
        configManager->writeConfigurationToJson(root, false);
        sendJsonResponse(request, 200, document);
    });

    // --- Configuration : modification en RAM -------------------------------
    auto* configHandler = new AsyncCallbackJsonWebHandler(
        "/api/config",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            String message;
            if (!configManager->applyConfigurationFromJson(json.as<JsonObjectConst>(), message)) {
                sendErrorResponse(request, 400, message);
                return;
            }
            robotController->requestConfigurationReload();
            sendSuccessResponse(request,
                                message.isEmpty()
                                    ? "Configuration appliquee. Utiliser Enregistrer pour la conserver."
                                    : message);
        });
    configHandler->setMethod(HTTP_POST);
    httpServer.addHandler(configHandler);

    // --- Configuration : enregistrement en memoire non volatile ------------
    httpServer.on("/api/config/save", HTTP_POST, [this](AsyncWebServerRequest* request) {
        if (configManager->saveConfiguration()) {
            sendSuccessResponse(request, "Configuration enregistree en memoire non volatile");
        } else {
            sendErrorResponse(request, 500, "Ecriture en memoire non volatile impossible");
        }
    });

    // --- Configuration : retour aux valeurs par defaut ---------------------
    httpServer.on("/api/config/reset", HTTP_POST, [this](AsyncWebServerRequest* request) {
        configManager->resetToDefaults();
        robotController->requestConfigurationReload();
        sendSuccessResponse(request,
                            "Valeurs par defaut restaurees. Enregistrer pour les rendre permanentes.");
    });

    // --- Journal -----------------------------------------------------------
    httpServer.on("/api/logs", HTTP_GET, [](AsyncWebServerRequest* request) {
        uint32_t sinceSequence = 0;
        if (request->hasParam("since")) {
            sinceSequence = strtoul(request->getParam("since")->value().c_str(), nullptr, 10);
        }

        static LogEntry entries[32];
        const int entryCount = Logger::copyEntriesSince(sinceSequence, entries, 32);

        JsonDocument document;
        JsonArray lines = document["lines"].to<JsonArray>();
        for (int entryIndex = 0; entryIndex < entryCount; ++entryIndex) {
            JsonObject line = lines.add<JsonObject>();
            line["seq"]     = entries[entryIndex].sequenceNumber;
            line["time"]    = entries[entryIndex].timestampMs;
            line["level"]   = Logger::levelName(entries[entryIndex].level);
            line["module"]  = entries[entryIndex].moduleName;
            line["message"] = entries[entryIndex].message;
        }
        document["lastSequence"] = Logger::lastSequenceNumber();
        sendJsonResponse(request, 200, document);
    });

    // --- Test moteur -------------------------------------------------------
    // Refuse par SafetyManager si le robot n'est pas DISARMED. La demande expire
    // d'elle-meme : l'interface doit la renouveler tant que le bouton est tenu.
    auto* motorTestHandler = new AsyncCallbackJsonWebHandler(
        "/api/motor-test",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            JsonObjectConst body = json.as<JsonObjectConst>();
            if (body.isNull()) {
                sendErrorResponse(request, 400, "Corps JSON attendu : {\"motor\":0,\"output\":0.2}");
                return;
            }

            const int   motorIndex = body["motor"]  | -1;
            const float output     = body["output"] | 0.0f;

            String message;
            if (!robotController->requestMotorTest(motorIndex, output, message)) {
                sendErrorResponse(request, 409, message);
                return;
            }
            sendSuccessResponse(request, "Test moteur en cours");
        });
    motorTestHandler->setMethod(HTTP_POST);
    httpServer.addHandler(motorTestHandler);

    httpServer.on("/api/motor-test/stop", HTTP_POST, [this](AsyncWebServerRequest* request) {
        robotController->cancelMotorTest();
        sendSuccessResponse(request, "Test moteur arrete");
    });

    // --- Arret d'urgence depuis l'interface --------------------------------
    httpServer.on("/api/disarm", HTTP_POST, [this](AsyncWebServerRequest* request) {
        robotController->requestDisarm();
        sendSuccessResponse(request, "Desarmement demande");
    });

    // --- Simulateur de mixer : ne commande AUCUN moteur --------------------
    auto* simulationHandler = new AsyncCallbackJsonWebHandler(
        "/api/mixer-simulate",
        [this](AsyncWebServerRequest* request, JsonVariant& json) {
            JsonObjectConst body = json.as<JsonObjectConst>();

            DriveCommand command;
            command.translationX = constrain(body["vx"]    | 0.0f, -1.0f, 1.0f);
            command.translationY = constrain(body["vy"]    | 0.0f, -1.0f, 1.0f);
            command.rotation     = constrain(body["omega"] | 0.0f, -1.0f, 1.0f);

            const RobotConfiguration& configuration = configManager->configuration();
            const WheelOutputs outputs =
                computeWheelCommands(command, configuration.wheels,
                                     configuration.drive.maxMotorOutput,
                                     configuration.drive.minMotorOutput,
                                     configuration.drive.normalizeTranslation);

            JsonDocument document;
            document["ok"] = true;
            JsonArray values = document["motors"].to<JsonArray>();
            for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
                values.add(outputs.values[wheelIndex]);
            }
            sendJsonResponse(request, 200, document);
        });
    simulationHandler->setMethod(HTTP_POST);
    httpServer.addHandler(simulationHandler);

    // --- Redemarrage -------------------------------------------------------
    httpServer.on("/api/reboot", HTTP_POST, [this](AsyncWebServerRequest* request) {
        LOG_WARN(logModule, "Redemarrage demande depuis l'interface Web");
        robotController->cancelMotorTest();
        request->send(200, "application/json", "{\"ok\":true,\"message\":\"Redemarrage\"}");
        // Laisse le temps a la reponse de partir avant de couper.
        httpServer.end();
        delay(200);
        ESP.restart();
    });

    // --- Fichiers statiques de l'interface ---------------------------------
    if (isFileSystemMounted) {
        httpServer.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    }

    httpServer.onNotFound([this](AsyncWebServerRequest* request) {
        if (request->url().startsWith("/api/")) {
            sendErrorResponse(request, 404, "Route d'API inconnue : " + request->url());
            return;
        }
        if (!isFileSystemMounted) {
            request->send(200, "text/html", missingFileSystemPage());
            return;
        }
        request->send(404, "text/plain", "Fichier introuvable");
    });
}

// -----------------------------------------------------------------------------
// Diffusion temps reel
// -----------------------------------------------------------------------------

void WebInterface::broadcastNewLogEntries()
{
    if (Logger::lastSequenceNumber() == lastSentLogSequence) {
        return;
    }

    LogEntry entries[16];
    const int entryCount = Logger::copyEntriesSince(lastSentLogSequence, entries, 16);
    if (entryCount == 0) {
        return;
    }

    JsonDocument document;
    document["type"] = "logs";
    JsonArray lines = document["lines"].to<JsonArray>();
    for (int entryIndex = 0; entryIndex < entryCount; ++entryIndex) {
        JsonObject line = lines.add<JsonObject>();
        line["seq"]     = entries[entryIndex].sequenceNumber;
        line["time"]    = entries[entryIndex].timestampMs;
        line["level"]   = Logger::levelName(entries[entryIndex].level);
        line["module"]  = entries[entryIndex].moduleName;
        line["message"] = entries[entryIndex].message;
        lastSentLogSequence = entries[entryIndex].sequenceNumber;
    }

    String payload;
    serializeJson(document, payload);
    webSocket.textAll(payload);
}

void WebInterface::update()
{
    webSocket.cleanupClients();

    const uint32_t nowMs = millis();
    if ((nowMs - lastTelemetryPushMs) < telemetryIntervalMs) {
        return;
    }
    lastTelemetryPushMs = nowMs;

    if (webSocket.count() == 0) {
        return;
    }

    JsonDocument document;
    JsonObject root = document.to<JsonObject>();
    root["type"] = "telemetry";
    buildTelemetryJson(root);

    String payload;
    serializeJson(document, payload);
    webSocket.textAll(payload);

    broadcastNewLogEntries();
}
