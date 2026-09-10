/**
 * @file config_manager.cpp
 *
 * @brief Valeurs par defaut, validation, persistance NVS et conversion JSON.
 */

#include "config_manager.h"

#include <Preferences.h>
#include <cstring>

#include "logger.h"
#include "omni_drive_mixer.h"

// Les identifiants Wi-Fi reels vivent dans un fichier exclu de Git. Son absence
// ne doit pas empecher la compilation : le robot demarre alors en point d'acces.
#if __has_include("wifi_secrets.h")
  #include "wifi_secrets.h"
#else
  #warning "include/wifi_secrets.h absent : demarrage en point d'acces. Voir wifi_secrets.example.h"
  #define WIFI_STATION_SSID            ""
  #define WIFI_STATION_PASSWORD        ""
  #define WIFI_ACCESS_POINT_PASSWORD   ""
  #define WIFI_DEFAULT_MODE_IS_STATION 0
#endif

namespace
{
    constexpr const char* logModule = "CONFIG";

    /// Espace de noms NVS et cles utilisees.
    constexpr const char* nvsNamespace  = "omnirobot";
    constexpr const char* nvsVersionKey = "version";

    /// Cle du document JSON de configuration.
    ///
    /// La configuration est volontairement stockee en JSON et non comme une
    /// copie binaire de RobotConfiguration : ajouter, retirer ou deplacer un
    /// champ de la structure changerait sa disposition memoire et rendrait
    /// illisible tout ce qui a ete enregistre auparavant. Avec le JSON, un
    /// champ nouveau prend simplement sa valeur par defaut et le reste des
    /// reglages de l'utilisateur est conserve.
    constexpr const char* nvsJsonKey = "configJson";

    /// Ancienne cle du format binaire, supprimee lors de la premiere sauvegarde.
    constexpr const char* legacyBlobKey = "config";

    /// Borne une valeur flottante dans un intervalle, en signalant toute correction.
    bool clampFloat(float& value, float minimum, float maximum)
    {
        if (isnan(value)) {
            value = minimum;
            return true;
        }
        if (value < minimum) {
            value = minimum;
            return true;
        }
        if (value > maximum) {
            value = maximum;
            return true;
        }
        return false;
    }

    bool clampInt(int& value, int minimum, int maximum)
    {
        if (value < minimum) {
            value = minimum;
            return true;
        }
        if (value > maximum) {
            value = maximum;
            return true;
        }
        return false;
    }

    bool clampUint32(uint32_t& value, uint32_t minimum, uint32_t maximum)
    {
        if (value < minimum) {
            value = minimum;
            return true;
        }
        if (value > maximum) {
            value = maximum;
            return true;
        }
        return false;
    }

    /// Copie une chaine dans un tampon de taille fixe, toujours zero terminee.
    void copyToFixedBuffer(char* destination, size_t destinationSize, const char* source)
    {
        strncpy(destination, source, destinationSize - 1);
        destination[destinationSize - 1] = '\0';
    }

    /// Valeurs par defaut communes aux trois axes de pilotage.
    AxisConfiguration makeDefaultAxis(int channelIndex, float expo)
    {
        AxisConfiguration axis{};
        axis.channelIndex = channelIndex;
        axis.inverted     = false;
        axis.rawMinimum   = ConfigDefaults::sbusRawMinimum;
        axis.rawCenter    = ConfigDefaults::sbusRawCenter;
        axis.rawMaximum   = ConfigDefaults::sbusRawMaximum;
        axis.deadband     = ConfigDefaults::deadband;
        axis.expo         = expo;
        axis.gain         = 1.0f;
        return axis;
    }

    bool validateAxis(AxisConfiguration& axis)
    {
        bool wasCorrected = false;
        wasCorrected |= clampInt(axis.channelIndex, 0, 15);
        // La borne haute couvre les deux protocoles : 2047 en SBUS, environ
        // 2200 microsecondes en iBUS pour les emetteurs a course etendue.
        wasCorrected |= clampInt(axis.rawMinimum, 0, 2500);
        wasCorrected |= clampInt(axis.rawCenter, 0, 2500);
        wasCorrected |= clampInt(axis.rawMaximum, 0, 2500);
        wasCorrected |= clampFloat(axis.deadband, 0.0f, 0.5f);
        wasCorrected |= clampFloat(axis.expo, 0.0f, 1.0f);
        wasCorrected |= clampFloat(axis.gain, 0.0f, 1.0f);

        // Une plage degeneree rendrait la normalisation instable ou nulle.
        if (axis.rawMaximum - axis.rawMinimum < 100) {
            axis.rawMinimum = ConfigDefaults::sbusRawMinimum;
            axis.rawMaximum = ConfigDefaults::sbusRawMaximum;
            wasCorrected = true;
        }
        if (axis.rawCenter <= axis.rawMinimum || axis.rawCenter >= axis.rawMaximum) {
            axis.rawCenter = (axis.rawMinimum + axis.rawMaximum) / 2;
            wasCorrected = true;
        }
        return wasCorrected;
    }
}

// -----------------------------------------------------------------------------
// Valeurs par defaut et validation (declarees dans robot_config.h)
// -----------------------------------------------------------------------------

void applyDefaultConfiguration(RobotConfiguration& configuration)
{
    configuration = RobotConfiguration{};
    configuration.version = robotConfigurationVersion;

    // --- Radio : CH1 = vx, CH2 = vy, CH4 = omega, CH5 = armement, CH6 = vitesse.
    configuration.radio.translationX = makeDefaultAxis(0, ConfigDefaults::translationExpo);
    configuration.radio.translationY = makeDefaultAxis(1, ConfigDefaults::translationExpo);
    configuration.radio.rotation     = makeDefaultAxis(3, ConfigDefaults::rotationExpo);

    configuration.radio.armChannelIndex           = 4;
    configuration.radio.armThresholdRawValue      = ConfigDefaults::sbusRawCenter;
    configuration.radio.armChannelIsInverted      = false;
    configuration.radio.speedModeChannelIndex     = 5;
    configuration.radio.speedModeThresholdRawValue = ConfigDefaults::sbusRawCenter;
    configuration.radio.frameTimeoutMs            = ConfigDefaults::sbusTimeoutMs;
    configuration.radio.protocol                  = RadioProtocol::Sbus;
    configuration.radio.autoDetectProtocol        = true;

    // --- Geometrie : angles symetriques, a confirmer sur le robot reel.
    applySymmetricWheelGeometry(configuration.wheels, ConfigDefaults::wheelGeometryAngleDeg);
    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        configuration.wheels[wheelIndex].outputGain = 1.0f;
        configuration.wheels[wheelIndex].inverted   = false;
        configuration.wheels[wheelIndex].enabled    = true;
    }

    // --- Pilotage : puissance volontairement limitee pour les premiers essais.
    configuration.drive.maxMotorOutput           = ConfigDefaults::maxMotorOutput;
    configuration.drive.minMotorOutput           = ConfigDefaults::minMotorOutput;
    configuration.drive.normalizeTranslation     = false;
    configuration.drive.maxTranslationOutput     = 1.0f;
    configuration.drive.maxRotationOutput        = 1.0f;
    configuration.drive.slowModeScale            = 0.40f;
    configuration.drive.maxAccelerationPerSecond = 2.0f;
    configuration.drive.maxDecelerationPerSecond = 4.0f;
    configuration.drive.pwmFrequencyHz           = ConfigDefaults::pwmFrequencyHz;
    configuration.drive.pwmResolutionBits        = ConfigDefaults::pwmResolutionBits;

    // --- Securite.
    configuration.safety.armingHoldMs           = 500;
    configuration.safety.armingNeutralTolerance = 0.10f;
    configuration.safety.motorTestMaxOutput     = 0.20f;
    configuration.safety.motorTestTimeoutMs     = 3000;

    // --- Wi-Fi : point d'acces par defaut, identifiants ajoutes par ConfigManager.
    configuration.wifi.mode = WifiMode::AccessPoint;
    configuration.wifi.accessPointSsid[0]     = '\0';
    configuration.wifi.accessPointPassword[0] = '\0';
    configuration.wifi.stationSsid[0]         = '\0';
    configuration.wifi.stationPassword[0]     = '\0';
    configuration.wifi.stationConnectTimeoutMs = 15000;
}

void applyProtocolCalibrationDefaults(RobotConfiguration& configuration, RadioProtocol protocol)
{
    const bool isIbus = (protocol == RadioProtocol::Ibus);

    const int rawMinimum = isIbus ? ConfigDefaults::ibusRawMinimum : ConfigDefaults::sbusRawMinimum;
    const int rawCenter  = isIbus ? ConfigDefaults::ibusRawCenter  : ConfigDefaults::sbusRawCenter;
    const int rawMaximum = isIbus ? ConfigDefaults::ibusRawMaximum : ConfigDefaults::sbusRawMaximum;

    AxisConfiguration* axes[] = {
        &configuration.radio.translationX,
        &configuration.radio.translationY,
        &configuration.radio.rotation,
    };

    for (AxisConfiguration* axis : axes) {
        axis->rawMinimum = rawMinimum;
        axis->rawCenter  = rawCenter;
        axis->rawMaximum = rawMaximum;
    }

    // Les seuils des voies de commutation suivent la meme echelle.
    configuration.radio.armThresholdRawValue       = rawCenter;
    configuration.radio.speedModeThresholdRawValue = rawCenter;

    configuration.radio.protocol = protocol;
}

bool clampConfigurationToValidRange(RobotConfiguration& configuration)
{
    bool wasCorrected = false;

    configuration.version = robotConfigurationVersion;

    wasCorrected |= validateAxis(configuration.radio.translationX);
    wasCorrected |= validateAxis(configuration.radio.translationY);
    wasCorrected |= validateAxis(configuration.radio.rotation);

    wasCorrected |= clampInt(configuration.radio.armChannelIndex, 0, 15);
    wasCorrected |= clampInt(configuration.radio.armThresholdRawValue, 0, 2500);
    wasCorrected |= clampInt(configuration.radio.speedModeChannelIndex, -1, 15);
    wasCorrected |= clampInt(configuration.radio.speedModeThresholdRawValue, 0, 2500);
    wasCorrected |= clampUint32(configuration.radio.frameTimeoutMs, 50, 2000);

    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        WheelConfiguration& wheel = configuration.wheels[wheelIndex];
        wasCorrected |= clampFloat(wheel.driveAngleDeg, -360.0f, 360.0f);
        wasCorrected |= clampFloat(wheel.rotationGain, -2.0f, 2.0f);
        wasCorrected |= clampFloat(wheel.outputGain, 0.0f, 1.0f);
    }

    wasCorrected |= clampFloat(configuration.drive.maxMotorOutput, 0.0f, 1.0f);
    // Un seuil de demarrage superieur a la moitie de l'echelle rendrait le
    // pilotage tout ou rien : ce n'est pas un mode de conduite acceptable.
    wasCorrected |= clampFloat(configuration.drive.minMotorOutput, 0.0f, 0.5f);
    wasCorrected |= clampFloat(configuration.drive.maxTranslationOutput, 0.0f, 1.0f);
    wasCorrected |= clampFloat(configuration.drive.maxRotationOutput, 0.0f, 1.0f);
    wasCorrected |= clampFloat(configuration.drive.slowModeScale, 0.05f, 1.0f);
    wasCorrected |= clampFloat(configuration.drive.maxAccelerationPerSecond, 0.0f, 50.0f);
    wasCorrected |= clampFloat(configuration.drive.maxDecelerationPerSecond, 0.0f, 50.0f);
    wasCorrected |= clampUint32(configuration.drive.pwmFrequencyHz, 500, 40000);

    int pwmResolutionBits = configuration.drive.pwmResolutionBits;
    if (clampInt(pwmResolutionBits, 8, 14)) {
        configuration.drive.pwmResolutionBits = static_cast<uint8_t>(pwmResolutionBits);
        wasCorrected = true;
    }

    wasCorrected |= clampUint32(configuration.safety.armingHoldMs, 0, 5000);
    wasCorrected |= clampFloat(configuration.safety.armingNeutralTolerance, 0.01f, 0.5f);
    // Le test moteur reste bride a 50 % quoi qu'il arrive : c'est un outil de
    // verification de cablage, pas un mode de pilotage.
    wasCorrected |= clampFloat(configuration.safety.motorTestMaxOutput, 0.0f, 0.5f);
    wasCorrected |= clampUint32(configuration.safety.motorTestTimeoutMs, 200, 10000);

    wasCorrected |= clampUint32(configuration.wifi.stationConnectTimeoutMs, 3000, 60000);

    if (configuration.radio.protocol != RadioProtocol::Sbus
        && configuration.radio.protocol != RadioProtocol::Ibus) {
        configuration.radio.protocol = RadioProtocol::Sbus;
        wasCorrected = true;
    }

    if (configuration.wifi.mode != WifiMode::AccessPoint
        && configuration.wifi.mode != WifiMode::Station) {
        configuration.wifi.mode = WifiMode::AccessPoint;
        wasCorrected = true;
    }

    // Un mot de passe de point d'acces trop court est refuse par la pile Wi-Fi :
    // mieux vaut un reseau ouvert qu'un point d'acces qui ne demarre pas.
    const size_t accessPointPasswordLength = strlen(configuration.wifi.accessPointPassword);
    if (accessPointPasswordLength > 0 && accessPointPasswordLength < 8) {
        configuration.wifi.accessPointPassword[0] = '\0';
        wasCorrected = true;
    }

    // Garantit que les tampons de chaines restent zero termines.
    configuration.wifi.accessPointSsid[wifiSsidBufferSize - 1]         = '\0';
    configuration.wifi.accessPointPassword[wifiPasswordBufferSize - 1] = '\0';
    configuration.wifi.stationSsid[wifiSsidBufferSize - 1]             = '\0';
    configuration.wifi.stationPassword[wifiPasswordBufferSize - 1]     = '\0';

    return wasCorrected;
}

// -----------------------------------------------------------------------------
// ConfigManager
// -----------------------------------------------------------------------------

void ConfigManager::applyCompiledWifiCredentials()
{
    // Les identifiants compiles ne servent que d'amorcage : des que l'utilisateur
    // enregistre une configuration reseau depuis l'interface Web, c'est la NVS
    // qui fait foi.
    if (currentConfiguration.wifi.stationSsid[0] == '\0' && strlen(WIFI_STATION_SSID) > 0) {
        copyToFixedBuffer(currentConfiguration.wifi.stationSsid,
                          wifiSsidBufferSize, WIFI_STATION_SSID);
        copyToFixedBuffer(currentConfiguration.wifi.stationPassword,
                          wifiPasswordBufferSize, WIFI_STATION_PASSWORD);

        if (WIFI_DEFAULT_MODE_IS_STATION) {
            currentConfiguration.wifi.mode = WifiMode::Station;
        }
        LOG_INFO(logModule, "Identifiants Wi-Fi compiles appliques (SSID '%s')",
                 currentConfiguration.wifi.stationSsid);
    }

    if (currentConfiguration.wifi.accessPointPassword[0] == '\0'
        && strlen(WIFI_ACCESS_POINT_PASSWORD) >= 8) {
        copyToFixedBuffer(currentConfiguration.wifi.accessPointPassword,
                          wifiPasswordBufferSize, WIFI_ACCESS_POINT_PASSWORD);
    }
}

bool ConfigManager::loadConfiguration(bool forceDefaults)
{
    applyDefaultConfiguration(currentConfiguration);

    if (forceDefaults) {
        LOG_WARN(logModule, "Restauration demandee : valeurs par defaut appliquees");
        applyCompiledWifiCredentials();
        clampConfigurationToValidRange(currentConfiguration);
        unsavedChanges = true;
        return false;
    }

    Preferences preferences;
    if (!preferences.begin(nvsNamespace, true)) {
        // Cas normal au tout premier demarrage : l'espace de noms n'existe pas
        // encore en NVS. Il sera cree a la premiere sauvegarde.
        LOG_INFO(logModule,
                 "Aucune configuration enregistree : valeurs par defaut appliquees");
        applyCompiledWifiCredentials();
        unsavedChanges = true;
        return false;
    }

    const uint32_t storedVersion = preferences.getUInt(nvsVersionKey, 0);
    const String   storedJson    = preferences.getString(nvsJsonKey, "");
    preferences.end();

    bool loadedFromNvs = false;

    if (storedJson.isEmpty()) {
        LOG_INFO(logModule, "Aucune configuration en NVS : valeurs par defaut");
    } else if (storedVersion > robotConfigurationVersion) {
        // Configuration ecrite par un firmware plus recent : son contenu peut
        // avoir une signification differente, on ne tente pas de l'interpreter.
        LOG_WARN(logModule,
                 "Configuration NVS en version %lu, plus recente que ce firmware (%lu) : "
                 "valeurs par defaut",
                 static_cast<unsigned long>(storedVersion),
                 static_cast<unsigned long>(robotConfigurationVersion));
    } else {
        JsonDocument storedDocument;
        const DeserializationError parseError = deserializeJson(storedDocument, storedJson);

        if (parseError) {
            LOG_ERROR(logModule,
                      "Configuration NVS illisible (%s) : valeurs par defaut appliquees",
                      parseError.c_str());
        } else {
            // La fusion se fait sur les valeurs par defaut deja en place : tout
            // champ absent du document conserve donc sa valeur par defaut, ce qui
            // rend le chargement tolerant aux evolutions de la configuration.
            String applyMessage;
            applyConfigurationFromJson(storedDocument.as<JsonObjectConst>(), applyMessage);
            loadedFromNvs = true;

            LOG_INFO(logModule, "Configuration chargee depuis la NVS (version %lu, %u octets)",
                     static_cast<unsigned long>(storedVersion),
                     static_cast<unsigned>(storedJson.length()));
            if (!applyMessage.isEmpty()) {
                LOG_WARN(logModule, "%s", applyMessage.c_str());
            }
        }
    }

    applyCompiledWifiCredentials();

    if (clampConfigurationToValidRange(currentConfiguration)) {
        LOG_WARN(logModule, "Des valeurs hors plage ont ete corrigees au chargement");
    }

    unsavedChanges = !loadedFromNvs;
    return loadedFromNvs;
}

bool ConfigManager::saveConfiguration()
{
    JsonDocument document;
    JsonObject root = document.to<JsonObject>();

    // Les mots de passe sont ecrits en clair : la NVS est une memoire locale a
    // la carte, et sans eux le robot ne pourrait pas rejoindre son reseau au
    // demarrage suivant. Ils ne sortent jamais par l'API Web.
    writeConfigurationToJson(root, true);

    String payload;
    serializeJson(document, payload);

    Preferences preferences;
    if (!preferences.begin(nvsNamespace, false)) {
        LOG_ERROR(logModule, "Ecriture NVS impossible : la configuration n'est pas enregistree");
        return false;
    }

    const size_t writtenBytes = preferences.putString(nvsJsonKey, payload);
    preferences.putUInt(nvsVersionKey, robotConfigurationVersion);

    // Menage : l'ancien format binaire n'a plus lieu d'occuper la NVS.
    preferences.remove(legacyBlobKey);
    preferences.remove("size");

    preferences.end();

    const bool wasWritten = (writtenBytes == payload.length());

    if (wasWritten) {
        unsavedChanges = false;
        LOG_INFO(logModule, "Configuration enregistree en NVS (%u octets JSON)",
                 static_cast<unsigned>(writtenBytes));
    } else {
        LOG_ERROR(logModule,
                  "Ecriture NVS incomplete : %u octets ecrits sur %u. "
                  "La configuration precedente est conservee.",
                  static_cast<unsigned>(writtenBytes),
                  static_cast<unsigned>(payload.length()));
    }

    return wasWritten;
}

void ConfigManager::resetToDefaults()
{
    RobotConfiguration defaults{};
    applyDefaultConfiguration(defaults);

    // Les identifiants Wi-Fi compiles sont reappliques sur la structure neuve.
    const RobotConfiguration previousConfiguration = currentConfiguration;
    currentConfiguration = defaults;
    applyCompiledWifiCredentials();
    defaults = currentConfiguration;
    currentConfiguration = previousConfiguration;

    replaceConfiguration(defaults);
    LOG_WARN(logModule, "Configuration reinitialisee : enregistrer pour rendre le changement permanent");
}

SemaphoreHandle_t ConfigManager::mutex() const
{
    if (configurationMutex == nullptr) {
        configurationMutex = xSemaphoreCreateMutex();
    }
    return configurationMutex;
}

bool ConfigManager::copyConfigurationTo(RobotConfiguration& destination, TickType_t waitTicks) const
{
    SemaphoreHandle_t lock = mutex();
    if (lock == nullptr) {
        return false;
    }
    if (xSemaphoreTake(lock, waitTicks) != pdTRUE) {
        return false;
    }

    destination = currentConfiguration;

    xSemaphoreGive(lock);
    return true;
}

bool ConfigManager::replaceConfiguration(const RobotConfiguration& candidate)
{
    RobotConfiguration validated = candidate;
    const bool wasCorrected = clampConfigurationToValidRange(validated);

    // Le remplacement se fait sous verrou : la tache de controle ne doit jamais
    // pouvoir lire une structure a moitie ecrite.
    SemaphoreHandle_t lock = mutex();
    if (lock != nullptr) {
        xSemaphoreTake(lock, portMAX_DELAY);
    }

    currentConfiguration = validated;
    unsavedChanges = true;

    if (lock != nullptr) {
        xSemaphoreGive(lock);
    }

    return wasCorrected;
}

// -----------------------------------------------------------------------------
// Conversion JSON
// -----------------------------------------------------------------------------

namespace
{
    void writeAxisToJson(JsonObject destination, const AxisConfiguration& axis)
    {
        destination["channel"]  = axis.channelIndex;
        destination["inverted"] = axis.inverted;
        destination["rawMin"]   = axis.rawMinimum;
        destination["rawCenter"]= axis.rawCenter;
        destination["rawMax"]   = axis.rawMaximum;
        destination["deadband"] = axis.deadband;
        destination["expo"]     = axis.expo;
        destination["gain"]     = axis.gain;
    }

    void readAxisFromJson(JsonObjectConst source, AxisConfiguration& axis)
    {
        if (source.isNull()) {
            return;
        }
        axis.channelIndex = source["channel"]   | axis.channelIndex;
        axis.inverted     = source["inverted"]  | axis.inverted;
        axis.rawMinimum   = source["rawMin"]    | axis.rawMinimum;
        axis.rawCenter    = source["rawCenter"] | axis.rawCenter;
        axis.rawMaximum   = source["rawMax"]    | axis.rawMaximum;
        axis.deadband     = source["deadband"]  | axis.deadband;
        axis.expo         = source["expo"]      | axis.expo;
        axis.gain         = source["gain"]      | axis.gain;
    }

    /**
     * @brief Copie un mot de passe recu du Web, en respectant le marqueur.
     *
     * Le marqueur signifie "ne pas changer" : il ne doit jamais etre enregistre
     * tel quel a la place du vrai mot de passe.
     */
    void readPasswordFromJson(JsonVariantConst source, char* destination, size_t destinationSize)
    {
        if (source.isNull() || !source.is<const char*>()) {
            return;
        }
        const char* received = source.as<const char*>();
        if (strcmp(received, maskedPasswordPlaceholder) == 0) {
            return;
        }
        copyToFixedBuffer(destination, destinationSize, received);
    }
}

void ConfigManager::writeConfigurationToJson(JsonObject destination, bool includeSecrets) const
{
    const RobotConfiguration& configuration = currentConfiguration;

    destination["version"] = configuration.version;

    JsonObject radio = destination["radio"].to<JsonObject>();
    writeAxisToJson(radio["translationX"].to<JsonObject>(), configuration.radio.translationX);
    writeAxisToJson(radio["translationY"].to<JsonObject>(), configuration.radio.translationY);
    writeAxisToJson(radio["rotation"].to<JsonObject>(),     configuration.radio.rotation);
    radio["armChannel"]         = configuration.radio.armChannelIndex;
    radio["armThreshold"]       = configuration.radio.armThresholdRawValue;
    radio["armInverted"]        = configuration.radio.armChannelIsInverted;
    radio["speedModeChannel"]   = configuration.radio.speedModeChannelIndex;
    radio["speedModeThreshold"] = configuration.radio.speedModeThresholdRawValue;
    radio["frameTimeoutMs"]     = configuration.radio.frameTimeoutMs;
    radio["protocol"]           = (configuration.radio.protocol == RadioProtocol::Ibus)
                                      ? "ibus" : "sbus";
    radio["autoDetectProtocol"] = configuration.radio.autoDetectProtocol;

    JsonArray wheels = destination["wheels"].to<JsonArray>();
    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        const WheelConfiguration& wheel = configuration.wheels[wheelIndex];
        JsonObject wheelObject = wheels.add<JsonObject>();
        wheelObject["angleDeg"]     = wheel.driveAngleDeg;
        wheelObject["rotationGain"] = wheel.rotationGain;
        wheelObject["outputGain"]   = wheel.outputGain;
        wheelObject["inverted"]     = wheel.inverted;
        wheelObject["enabled"]      = wheel.enabled;
    }

    JsonObject drive = destination["drive"].to<JsonObject>();
    drive["maxMotorOutput"]       = configuration.drive.maxMotorOutput;
    drive["minMotorOutput"]       = configuration.drive.minMotorOutput;
    drive["normalizeTranslation"] = configuration.drive.normalizeTranslation;
    drive["maxTranslationOutput"] = configuration.drive.maxTranslationOutput;
    drive["maxRotationOutput"]    = configuration.drive.maxRotationOutput;
    drive["slowModeScale"]        = configuration.drive.slowModeScale;
    drive["maxAcceleration"]      = configuration.drive.maxAccelerationPerSecond;
    drive["maxDeceleration"]      = configuration.drive.maxDecelerationPerSecond;
    drive["pwmFrequencyHz"]       = configuration.drive.pwmFrequencyHz;
    drive["pwmResolutionBits"]    = configuration.drive.pwmResolutionBits;

    JsonObject safety = destination["safety"].to<JsonObject>();
    safety["armingHoldMs"]        = configuration.safety.armingHoldMs;
    safety["armingNeutralTolerance"] = configuration.safety.armingNeutralTolerance;
    safety["motorTestMaxOutput"]  = configuration.safety.motorTestMaxOutput;
    safety["motorTestTimeoutMs"]  = configuration.safety.motorTestTimeoutMs;

    JsonObject wifi = destination["wifi"].to<JsonObject>();
    wifi["mode"]                 = (configuration.wifi.mode == WifiMode::Station) ? "station" : "ap";
    wifi["apSsid"]               = configuration.wifi.accessPointSsid;
    wifi["stationSsid"]          = configuration.wifi.stationSsid;
    wifi["connectTimeoutMs"]     = configuration.wifi.stationConnectTimeoutMs;

    // Les mots de passe ne quittent le firmware en clair que sur demande
    // explicite, ce qui n'arrive jamais pour une reponse HTTP.
    if (includeSecrets) {
        wifi["apPassword"]      = configuration.wifi.accessPointPassword;
        wifi["stationPassword"] = configuration.wifi.stationPassword;
    } else {
        wifi["apPassword"] = (configuration.wifi.accessPointPassword[0] != '\0')
                                 ? maskedPasswordPlaceholder : "";
        wifi["stationPassword"] = (configuration.wifi.stationPassword[0] != '\0')
                                 ? maskedPasswordPlaceholder : "";
    }
}

bool ConfigManager::applyConfigurationFromJson(JsonObjectConst source, String& errorMessage)
{
    if (source.isNull()) {
        errorMessage = "Document JSON vide ou invalide";
        return false;
    }

    // La modification porte sur une copie : la configuration active n'est
    // remplacee qu'une fois le document entierement lu et valide.
    RobotConfiguration candidate = currentConfiguration;

    JsonObjectConst radio = source["radio"];
    if (!radio.isNull()) {
        // Le protocole est traite AVANT les axes. Changer de protocole remet la
        // calibration aux plages du nouveau, mais un document qui fournit aussi
        // ses propres valeurs d'axes doit pouvoir les imposer ensuite : c'est le
        // cas du chargement depuis la NVS, ou la calibration de l'utilisateur
        // serait sinon ecrasee par les valeurs par defaut a chaque demarrage.
        candidate.radio.autoDetectProtocol =
            radio["autoDetectProtocol"] | candidate.radio.autoDetectProtocol;

        const char* protocolText = radio["protocol"] | "";
        if (strcmp(protocolText, "ibus") == 0 && candidate.radio.protocol != RadioProtocol::Ibus) {
            applyProtocolCalibrationDefaults(candidate, RadioProtocol::Ibus);
        } else if (strcmp(protocolText, "sbus") == 0
                   && candidate.radio.protocol != RadioProtocol::Sbus) {
            applyProtocolCalibrationDefaults(candidate, RadioProtocol::Sbus);
        }

        readAxisFromJson(radio["translationX"], candidate.radio.translationX);
        readAxisFromJson(radio["translationY"], candidate.radio.translationY);
        readAxisFromJson(radio["rotation"],     candidate.radio.rotation);
        candidate.radio.armChannelIndex      = radio["armChannel"]   | candidate.radio.armChannelIndex;
        candidate.radio.armThresholdRawValue = radio["armThreshold"] | candidate.radio.armThresholdRawValue;
        candidate.radio.armChannelIsInverted = radio["armInverted"]  | candidate.radio.armChannelIsInverted;
        candidate.radio.speedModeChannelIndex =
            radio["speedModeChannel"] | candidate.radio.speedModeChannelIndex;
        candidate.radio.speedModeThresholdRawValue =
            radio["speedModeThreshold"] | candidate.radio.speedModeThresholdRawValue;
        candidate.radio.frameTimeoutMs = radio["frameTimeoutMs"] | candidate.radio.frameTimeoutMs;
    }

    JsonArrayConst wheels = source["wheels"];
    if (!wheels.isNull()) {
        if (wheels.size() != static_cast<size_t>(wheelCount)) {
            errorMessage = "Le tableau 'wheels' doit contenir exactement 4 elements";
            return false;
        }
        for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
            JsonObjectConst wheelObject = wheels[wheelIndex];
            WheelConfiguration& wheel = candidate.wheels[wheelIndex];
            wheel.driveAngleDeg = wheelObject["angleDeg"]     | wheel.driveAngleDeg;
            wheel.rotationGain  = wheelObject["rotationGain"] | wheel.rotationGain;
            wheel.outputGain    = wheelObject["outputGain"]   | wheel.outputGain;
            wheel.inverted      = wheelObject["inverted"]     | wheel.inverted;
            wheel.enabled       = wheelObject["enabled"]      | wheel.enabled;
        }
    }

    JsonObjectConst drive = source["drive"];
    if (!drive.isNull()) {
        candidate.drive.maxMotorOutput = drive["maxMotorOutput"] | candidate.drive.maxMotorOutput;
        candidate.drive.minMotorOutput = drive["minMotorOutput"] | candidate.drive.minMotorOutput;
        candidate.drive.normalizeTranslation =
            drive["normalizeTranslation"] | candidate.drive.normalizeTranslation;
        candidate.drive.maxTranslationOutput =
            drive["maxTranslationOutput"] | candidate.drive.maxTranslationOutput;
        candidate.drive.maxRotationOutput =
            drive["maxRotationOutput"] | candidate.drive.maxRotationOutput;
        candidate.drive.slowModeScale = drive["slowModeScale"] | candidate.drive.slowModeScale;
        candidate.drive.maxAccelerationPerSecond =
            drive["maxAcceleration"] | candidate.drive.maxAccelerationPerSecond;
        candidate.drive.maxDecelerationPerSecond =
            drive["maxDeceleration"] | candidate.drive.maxDecelerationPerSecond;
        candidate.drive.pwmFrequencyHz = drive["pwmFrequencyHz"] | candidate.drive.pwmFrequencyHz;
        candidate.drive.pwmResolutionBits =
            drive["pwmResolutionBits"] | candidate.drive.pwmResolutionBits;
    }

    JsonObjectConst safety = source["safety"];
    if (!safety.isNull()) {
        candidate.safety.armingHoldMs = safety["armingHoldMs"] | candidate.safety.armingHoldMs;
        candidate.safety.armingNeutralTolerance =
            safety["armingNeutralTolerance"] | candidate.safety.armingNeutralTolerance;
        candidate.safety.motorTestMaxOutput =
            safety["motorTestMaxOutput"] | candidate.safety.motorTestMaxOutput;
        candidate.safety.motorTestTimeoutMs =
            safety["motorTestTimeoutMs"] | candidate.safety.motorTestTimeoutMs;
    }

    JsonObjectConst wifi = source["wifi"];
    if (!wifi.isNull()) {
        const char* modeText = wifi["mode"] | "";
        if (strcmp(modeText, "station") == 0) {
            candidate.wifi.mode = WifiMode::Station;
        } else if (strcmp(modeText, "ap") == 0) {
            candidate.wifi.mode = WifiMode::AccessPoint;
        }

        if (wifi["apSsid"].is<const char*>()) {
            copyToFixedBuffer(candidate.wifi.accessPointSsid, wifiSsidBufferSize,
                              wifi["apSsid"].as<const char*>());
        }
        if (wifi["stationSsid"].is<const char*>()) {
            copyToFixedBuffer(candidate.wifi.stationSsid, wifiSsidBufferSize,
                              wifi["stationSsid"].as<const char*>());
        }
        readPasswordFromJson(wifi["apPassword"], candidate.wifi.accessPointPassword,
                             wifiPasswordBufferSize);
        readPasswordFromJson(wifi["stationPassword"], candidate.wifi.stationPassword,
                             wifiPasswordBufferSize);

        candidate.wifi.stationConnectTimeoutMs =
            wifi["connectTimeoutMs"] | candidate.wifi.stationConnectTimeoutMs;
    }

    if (replaceConfiguration(candidate)) {
        errorMessage = "Configuration appliquee, mais des valeurs hors plage ont ete corrigees";
    }
    return true;
}
