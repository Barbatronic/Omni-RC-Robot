/**
 * @file logger.cpp
 *
 * @brief Implementation du journal : sortie serie et tampon circulaire RAM.
 *
 * Le tampon est protege par un mutex FreeRTOS car il est ecrit depuis la tache
 * de controle moteur et lu depuis la tache reseau.
 */

#include "logger.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace
{
    LogEntry          entries[logBufferLineCount];
    int               nextWriteIndex   = 0;
    uint32_t          sequenceCounter  = 0;
    LogLevel          configuredLevel  = LogLevel::Info;
    SemaphoreHandle_t bufferMutex      = nullptr;
    bool              isInitialized    = false;

    /// Prend le mutex si le logger est pret. Retourne false si le tampon est inaccessible.
    bool lockBuffer()
    {
        if (bufferMutex == nullptr) {
            return false;
        }
        return xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(5)) == pdTRUE;
    }

    void unlockBuffer()
    {
        xSemaphoreGive(bufferMutex);
    }
}

void Logger::begin(unsigned long serialBaudRate, LogLevel minimumLevel)
{
    configuredLevel = minimumLevel;

    if (bufferMutex == nullptr) {
        bufferMutex = xSemaphoreCreateMutex();
    }

    Serial.begin(serialBaudRate);

    // Laisse le temps a l'hote d'ouvrir le port, sans bloquer indefiniment si
    // aucun terminal n'est connecte (cas normal du robot alimente sur batterie).
    const uint32_t serialWaitStartMs = millis();
    while (!Serial && (millis() - serialWaitStartMs) < 1500) {
        delay(10);
    }

    isInitialized = true;
}

void Logger::setMinimumLevel(LogLevel minimumLevel)
{
    configuredLevel = minimumLevel;
}

LogLevel Logger::minimumLevel()
{
    return configuredLevel;
}

const char* Logger::levelName(LogLevel level)
{
    switch (level) {
        case LogLevel::Error: return "ERROR";
        case LogLevel::Warn:  return "WARN";
        case LogLevel::Info:  return "INFO";
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Trace: return "TRACE";
    }
    return "?";
}

void Logger::log(LogLevel level, const char* moduleName, const char* format, ...)
{
    if (static_cast<uint8_t>(level) > static_cast<uint8_t>(configuredLevel)) {
        return;
    }

    char formattedMessage[logBufferLineLength];

    va_list arguments;
    va_start(arguments, format);
    vsnprintf(formattedMessage, sizeof(formattedMessage), format, arguments);
    va_end(arguments);

    const uint32_t timestampMs = millis();

    if (isInitialized) {
        // Format : [secondes.millisecondes][NIVEAU][MODULE] message
        Serial.printf("[%6lu.%03lu][%s][%s] %s\r\n",
                      static_cast<unsigned long>(timestampMs / 1000),
                      static_cast<unsigned long>(timestampMs % 1000),
                      levelName(level),
                      moduleName,
                      formattedMessage);
    }

    if (!lockBuffer()) {
        return;
    }

    LogEntry& entry = entries[nextWriteIndex];
    entry.sequenceNumber = ++sequenceCounter;
    entry.timestampMs    = timestampMs;
    entry.level          = level;
    strncpy(entry.moduleName, moduleName, sizeof(entry.moduleName) - 1);
    entry.moduleName[sizeof(entry.moduleName) - 1] = '\0';
    strncpy(entry.message, formattedMessage, sizeof(entry.message) - 1);
    entry.message[sizeof(entry.message) - 1] = '\0';

    nextWriteIndex = (nextWriteIndex + 1) % logBufferLineCount;

    unlockBuffer();
}

int Logger::copyEntriesSince(uint32_t afterSequenceNumber, LogEntry* destination, int maximumEntries)
{
    if (destination == nullptr || maximumEntries <= 0 || !lockBuffer()) {
        return 0;
    }

    int copiedCount = 0;

    // Parcourt le tampon de la plus ancienne a la plus recente entree.
    for (int offset = 0; offset < logBufferLineCount && copiedCount < maximumEntries; ++offset) {
        const LogEntry& entry = entries[(nextWriteIndex + offset) % logBufferLineCount];

        if (entry.sequenceNumber > afterSequenceNumber) {
            destination[copiedCount] = entry;
            ++copiedCount;
        }
    }

    unlockBuffer();
    return copiedCount;
}

uint32_t Logger::lastSequenceNumber()
{
    return sequenceCounter;
}
