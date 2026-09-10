#pragma once

/**
 * @file logger.h
 *
 * @brief Journalisation centralisee du firmware.
 *
 * Entrees  : messages emis par tous les modules.
 * Sorties  : terminal serie USB, et tampon circulaire lu par l'interface Web.
 *
 * Le logger est volontairement minimal : il n'alloue pas de memoire dynamique
 * et reste utilisable depuis la tache temps reel de controle moteur.
 *
 * Regle d'usage : ne jamais journaliser a chaque iteration de la boucle de
 * controle (200 Hz), sous peine de saturer le terminal et le tampon.
 */

#include <Arduino.h>

/// Niveaux de gravite, du plus grave au plus verbeux.
enum class LogLevel : uint8_t
{
    Error = 0,
    Warn  = 1,
    Info  = 2,
    Debug = 3,
    Trace = 4,
};

/// Nombre de lignes conservees en RAM pour l'interface Web.
constexpr int logBufferLineCount  = 160;
/// Longueur maximale d'une ligne, zero terminal compris.
constexpr int logBufferLineLength = 128;

/**
 * @brief Une ligne de journal conservee en memoire.
 */
struct LogEntry
{
    uint32_t sequenceNumber;  ///< Identifiant croissant, permet a l'interface de ne lire que le nouveau
    uint32_t timestampMs;     ///< millis() au moment de l'emission
    LogLevel level;
    char     moduleName[12];
    char     message[logBufferLineLength];
};

namespace Logger
{
    /**
     * @brief Initialise le port serie et le tampon circulaire.
     *
     * @param serialBaudRate Vitesse du terminal serie.
     * @param minimumLevel Niveau minimal reellement journalise.
     */
    void begin(unsigned long serialBaudRate, LogLevel minimumLevel = LogLevel::Info);

    /// Change le niveau minimal journalise.
    void setMinimumLevel(LogLevel minimumLevel);
    LogLevel minimumLevel();

    /**
     * @brief Journalise un message formate facon printf.
     *
     * Sur pour un appel depuis n'importe quelle tache FreeRTOS.
     *
     * @param level Gravite du message.
     * @param moduleName Nom court du module emetteur, par exemple "RADIO".
     * @param format Chaine de format printf.
     */
    void log(LogLevel level, const char* moduleName, const char* format, ...)
        __attribute__((format(printf, 3, 4)));

    /**
     * @brief Copie les lignes de journal plus recentes qu'un numero de sequence.
     *
     * @param afterSequenceNumber Ne retourne que les lignes strictement posterieures.
     * @param destination Tableau de destination fourni par l'appelant.
     * @param maximumEntries Capacite de @p destination.
     * @return Nombre de lignes ecrites dans @p destination.
     */
    int copyEntriesSince(uint32_t afterSequenceNumber, LogEntry* destination, int maximumEntries);

    /// Numero de sequence de la derniere ligne emise.
    uint32_t lastSequenceNumber();

    /// Nom court d'un niveau, pour l'affichage ("ERROR", "WARN", ...).
    const char* levelName(LogLevel level);
}

// Raccourcis utilises dans tout le projet.
#define LOG_ERROR(module, ...) Logger::log(LogLevel::Error, module, __VA_ARGS__)
#define LOG_WARN(module, ...)  Logger::log(LogLevel::Warn,  module, __VA_ARGS__)
#define LOG_INFO(module, ...)  Logger::log(LogLevel::Info,  module, __VA_ARGS__)
#define LOG_DEBUG(module, ...) Logger::log(LogLevel::Debug, module, __VA_ARGS__)
#define LOG_TRACE(module, ...) Logger::log(LogLevel::Trace, module, __VA_ARGS__)
