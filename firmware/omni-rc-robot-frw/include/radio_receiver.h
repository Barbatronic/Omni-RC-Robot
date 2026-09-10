#pragma once

/**
 * @file radio_receiver.h
 *
 * @brief Reception et decodage de la radiocommande.
 *
 * Entrees  : trames serie emises par le recepteur RC sur une UART dediee.
 * Sorties  : voies brutes, indicateurs de perte de liaison, statistiques.
 * Dialogue avec : RobotController (lecture des voies), SafetyManager (etat radio).
 *
 * ---------------------------------------------------------------------------
 * Deux protocoles supportes
 * ---------------------------------------------------------------------------
 *
 * SBUS (Futaba, FrSky...)
 *   100000 bauds, 8 bits, parite paire, 2 stops, signal electriquement INVERSE.
 *   Trame de 25 octets : 0x0F, 22 octets de donnees (16 voies de 11 bits),
 *   1 octet d'indicateurs (frame lost / failsafe), 1 octet de fin.
 *   Voies brutes typiquement entre 172 et 1811.
 *
 * iBUS (FlySky : FS-iA6B, FS-iA10B...)
 *   115200 bauds, 8 bits, sans parite, 1 stop, signal NON inverse.
 *   Trame de 32 octets : 0x20, 0x40, 14 voies de 16 bits petit-boutiste,
 *   somme de controle sur 16 bits.
 *   Voies brutes directement en microsecondes, typiquement entre 1000 et 2000.
 *   Le protocole ne transporte aucun indicateur de failsafe : la perte de
 *   liaison est detectee uniquement par le chien de garde temporel.
 *
 * L'UART de l'ESP32-S3 sait inverser elle-meme le signal RX : aucun inverseur
 * externe n'est necessaire, quel que soit le protocole.
 *
 * Le protocole peut etre impose par la configuration, ou reconnu automatiquement
 * au demarrage en essayant chaque reglage jusqu'a obtenir des trames valides.
 */

#include <Arduino.h>

#include "robot_config.h"

/// Nombre de voies exposees au reste du firmware.
constexpr int radioChannelCount = 16;

/// Nombre d'octets bruts conserves pour le diagnostic de cablage.
constexpr int radioRawDumpSize = 25;

/**
 * @brief Etat de la liaison radio a un instant donne.
 */
struct RadioStatus
{
    RadioProtocol protocol;          ///< Protocole reellement utilise
    bool     isConnected;            ///< Une trame valide a ete recue recemment
    bool     isFailsafeActive;       ///< Le recepteur signale une perte de liaison
    bool     isFrameLost;            ///< La derniere trame signale une perte ponctuelle
    uint32_t lastValidFrameTimeMs;   ///< millis() de la derniere trame valide
    uint32_t frameAgeMs;             ///< Age de cette derniere trame
    float    frameRateHz;            ///< Cadence approximative des trames
    uint32_t validFrameCount;        ///< Compteur total de trames valides
    uint32_t decodingErrorCount;     ///< Compteur total de trames rejetees
    uint32_t frameLostCount;         ///< Compteur total d'indicateurs frameLost
    uint32_t rawByteCount;           ///< Octets bruts lus sur l'UART depuis le demarrage
};

/// Nom lisible d'un protocole, pour les logs et l'interface Web.
const char* radioProtocolName(RadioProtocol protocol);

/**
 * @brief Lit et decode le flux serie du recepteur RC.
 */
class RadioReceiver
{
public:
    /**
     * @brief Configure l'UART dediee a la radiocommande.
     *
     * @param uartNumber Numero d'UART materielle (1 ou 2).
     * @param receivePin GPIO relie a la sortie du recepteur.
     * @param frameTimeoutMs Duree sans trame valide au-dela de laquelle la
     *        liaison est declaree perdue.
     * @param protocol Protocole a utiliser.
     */
    void begin(int uartNumber, int receivePin, uint32_t frameTimeoutMs, RadioProtocol protocol);

    /// Reconfigure l'UART pour le protocole demande et vide le tampon.
    void applyProtocol(RadioProtocol protocol);

    /**
     * @brief Cherche le protocole qui produit des trames valides.
     *
     * Essaie le protocole demande en premier, puis les autres. Sans emetteur
     * allume, aucun ne donnera de resultat : le protocole demande est alors
     * conserve, ce qui n'est pas une erreur.
     *
     * @param preferredProtocol Protocole essaye en premier.
     * @param listenDurationMs Duree d'ecoute pour chaque candidat.
     * @return true si un protocole produisant des trames valides a ete trouve.
     */
    bool detectProtocol(RadioProtocol preferredProtocol, uint32_t listenDurationMs);

    /**
     * @brief Consomme les octets disponibles et decode les trames completes.
     *
     * A appeler a chaque iteration de la boucle de controle. Ne bloque jamais.
     *
     * @return true si une nouvelle trame valide a ete decodee pendant cet appel.
     */
    bool readIncomingFrames();

    /// Met a jour le delai de perte de liaison sans reinitialiser l'UART.
    void setFrameTimeoutMs(uint32_t frameTimeoutMs);

    /// Etat courant de la liaison radio.
    RadioStatus status() const;

    /// Protocole reellement utilise.
    RadioProtocol activeProtocol() const { return protocol; }

    /**
     * @brief Valeur brute d'une voie.
     *
     * @param channelIndex Index de la voie, de 0 (CH1) a 15 (CH16).
     * @return Valeur brute, ou 0 si l'index est hors plage.
     */
    uint16_t rawChannel(int channelIndex) const;

    /// Copie les 16 voies brutes dans @p destination.
    void copyRawChannels(uint16_t* destination) const;

    /**
     * @brief Copie les derniers octets recus, pour diagnostiquer le cablage.
     *
     * Permet de distinguer trois situations lors de la mise au point :
     *  - aucun octet        : rien n'est emis, ou le fil n'est pas sur la bonne broche ;
     *  - octets incoherents : protocole, vitesse ou polarite incorrects ;
     *  - octets structures  : trames presentes, entete reconnaissable.
     *
     * @param destination Tableau d'au moins radioRawDumpSize octets.
     * @return Nombre d'octets ecrits.
     */
    int copyRawByteDump(uint8_t* destination) const;

private:
    /// Ajoute un octet au tampon de trame et decode si la trame est complete.
    bool consumeByte(uint8_t incomingByte, uint32_t nowMicroseconds);

    /// Decode une trame SBUS de 25 octets deja accumulee.
    bool decodeSbusFrame();

    /// Decode une trame iBUS de 32 octets deja accumulee.
    bool decodeIbusFrame();

    /// Enregistre une trame valide et met a jour la cadence mesuree.
    void registerValidFrame();

    /// Longueur de trame du protocole courant.
    int expectedFrameLength() const;

    static constexpr int maximumFrameLength = 32;   ///< iBUS, le plus long des deux

    /// Un silence superieur a cette duree signale une frontiere entre deux trames.
    static constexpr uint32_t interFrameGapMicroseconds = 2000;

    HardwareSerial* serialPort = nullptr;
    int             configuredUartNumber = 1;
    int             configuredReceivePin = -1;
    RadioProtocol   protocol             = RadioProtocol::Sbus;

    uint8_t  frameBuffer[maximumFrameLength] = {};
    int      frameByteCount               = 0;
    uint32_t lastByteTimeMicroseconds     = 0;

    uint16_t channels[radioChannelCount] = {};
    bool     failsafeFlag               = false;
    bool     frameLostFlag              = false;

    uint32_t frameTimeoutMs         = ConfigDefaults::sbusTimeoutMs;
    uint32_t lastValidFrameTimeMs   = 0;
    uint32_t validFrameCount        = 0;
    uint32_t decodingErrorCount     = 0;
    uint32_t frameLostCount         = 0;
    uint32_t rawByteCount           = 0;
    uint8_t  rawByteDump[radioRawDumpSize] = {};
    int      rawByteDumpIndex       = 0;
    float    measuredFrameRateHz    = 0.0f;
    bool     hasReceivedAnyFrame    = false;
};

/**
 * @brief Convertit une voie brute en valeur normalisee et conditionnee.
 *
 * Applique successivement : calibration min/centre/max, normalisation sur
 * -1..+1, inversion eventuelle, zone morte remise a l'echelle, courbe
 * exponentielle, puis gain utilisateur.
 *
 * Fonction pure : elle ne depend d'aucun materiel et peut etre testee seule.
 * Elle est independante du protocole, la calibration absorbant la difference
 * de plage entre SBUS et iBUS.
 *
 * @param rawValue Valeur brute recue du recepteur.
 * @param axis Calibration et traitement de l'axe.
 * @return Valeur comprise entre -1.0 et +1.0.
 */
float normalizeRadioChannel(uint16_t rawValue, const AxisConfiguration& axis);
