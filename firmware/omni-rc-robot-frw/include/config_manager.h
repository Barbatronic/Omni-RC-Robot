#pragma once

/**
 * @file config_manager.h
 *
 * @brief Chargement, validation et sauvegarde de la configuration du robot.
 *
 * Entrees  : memoire non volatile (NVS), documents JSON envoyes par l'interface Web,
 *            identifiants Wi-Fi compiles (wifi_secrets.h).
 * Sorties  : une structure RobotConfiguration validee, utilisee par tous les modules.
 * Dialogue avec : RobotController et WebInterface.
 *
 * La configuration vit en RAM pendant le fonctionnement. L'ecriture en flash
 * n'a lieu que sur demande explicite, afin de ne pas user la NVS a chaque
 * deplacement d'un curseur dans l'interface Web.
 *
 * ---------------------------------------------------------------------------
 * Traitement des mots de passe Wi-Fi
 * ---------------------------------------------------------------------------
 * Les mots de passe sont stockes en NVS mais ne sortent JAMAIS en clair par
 * l'API Web : ils sont remplaces par un marqueur. Renvoyer ce marqueur lors
 * d'un enregistrement conserve le mot de passe deja en place.
 */

#include <Arduino.h>
#include <ArduinoJson.h>

#include "robot_config.h"

/// Marqueur substitue aux mots de passe dans les reponses de l'API Web.
constexpr const char* maskedPasswordPlaceholder = "********";

/**
 * @brief Detient la configuration courante et gere sa persistance.
 */
class ConfigManager
{
public:
    /**
     * @brief Charge la configuration depuis la NVS, ou applique les defauts.
     *
     * @param forceDefaults true pour ignorer la NVS et repartir des valeurs par
     *        defaut, par exemple lorsque le bouton de reset est maintenu au boot.
     * @return true si une configuration valide a ete lue en NVS.
     */
    bool loadConfiguration(bool forceDefaults = false);

    /// Ecrit la configuration courante en NVS.
    bool saveConfiguration();

    /// Restaure les valeurs par defaut en RAM, sans ecrire en flash.
    void resetToDefaults();

    /// Configuration courante, en lecture seule.
    ///
    /// Reservee a la tache reseau, qui est aussi la seule a modifier la
    /// configuration. La tache de controle doit utiliser copyConfigurationTo().
    const RobotConfiguration& configuration() const { return currentConfiguration; }

    /**
     * @brief Copie la configuration sous verrou, pour une autre tache.
     *
     * La tache de controle moteur ne doit jamais lire directement la structure :
     * la tache reseau pourrait etre en train de la remplacer, ce qui donnerait
     * une configuration incoherente, melangeant ancienne et nouvelle valeurs.
     *
     * @param destination Structure de destination.
     * @param waitTicks Attente maximale du verrou. Zero pour ne jamais bloquer.
     * @return true si la copie a eu lieu.
     */
    bool copyConfigurationTo(RobotConfiguration& destination, TickType_t waitTicks = 0) const;

    /**
     * @brief Remplace la configuration courante apres validation.
     *
     * @param candidate Configuration proposee ; elle est bornee avant adoption.
     * @return true si au moins une valeur a du etre corrigee.
     */
    bool replaceConfiguration(const RobotConfiguration& candidate);

    /**
     * @brief Serialise la configuration vers un objet JSON.
     *
     * @param destination Objet JSON a remplir.
     * @param includeSecrets true pour ecrire les mots de passe en clair.
     *        Doit rester false pour toute reponse envoyee sur le reseau.
     */
    void writeConfigurationToJson(JsonObject destination, bool includeSecrets = false) const;

    /**
     * @brief Applique un document JSON partiel a la configuration courante.
     *
     * Seules les cles presentes sont modifiees ; le resultat est valide et borne.
     * Un mot de passe egal au marqueur laisse la valeur existante inchangee.
     *
     * @param source Objet JSON recu de l'interface Web.
     * @param errorMessage Renseigne avec une explication en cas d'echec.
     * @return true si le document a ete applique.
     */
    bool applyConfigurationFromJson(JsonObjectConst source, String& errorMessage);

    /// true si la configuration en RAM differe de celle enregistree en NVS.
    bool hasUnsavedChanges() const { return unsavedChanges; }

private:
    /// Renseigne les identifiants Wi-Fi compiles s'ils sont absents de la config.
    void applyCompiledWifiCredentials();

    /// Cree le verrou au premier usage et le retourne.
    SemaphoreHandle_t mutex() const;

    RobotConfiguration        currentConfiguration{};
    bool                      unsavedChanges  = false;
    mutable SemaphoreHandle_t configurationMutex = nullptr;
};
