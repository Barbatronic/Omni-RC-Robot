/**
 * @file radio_receiver.cpp
 *
 * @brief Decodage des protocoles SBUS et iBUS, et conditionnement des voies.
 *
 * La synchronisation des trames repose sur deux criteres complementaires : le
 * silence entre deux trames et la reconnaissance de l'entete du protocole. Cela
 * evite de rester cale sur un mauvais alignement apres une perturbation.
 *
 * Chaque protocole valide ses trames par un critere qui lui est propre :
 * octet de fin pour le SBUS, somme de controle sur 16 bits pour l'iBUS.
 */

#include "radio_receiver.h"

#include "logger.h"

#include <cmath>
#include <cstring>

namespace
{
    constexpr const char* logModule = "RADIO";

    /// Constante de lissage de la mesure de cadence des trames.
    constexpr float frameRateSmoothingFactor = 0.1f;

    // --- SBUS ---------------------------------------------------------------
    constexpr int     sbusFrameLength = 25;
    constexpr uint8_t sbusHeaderByte  = 0x0F;

    // --- iBUS ---------------------------------------------------------------
    constexpr int     ibusFrameLength   = 32;
    constexpr uint8_t ibusLengthByte    = 0x20;  ///< Longueur de trame, sert d'entete
    constexpr uint8_t ibusCommandByte   = 0x40;  ///< Commande "voies RC"
    constexpr int     ibusChannelCount  = 14;

    HardwareSerial radioSerialPort1(1);
    HardwareSerial radioSerialPort2(2);
}

const char* radioProtocolName(RadioProtocol protocol)
{
    switch (protocol) {
        case RadioProtocol::Sbus: return "SBUS";
        case RadioProtocol::Ibus: return "iBUS";
    }
    return "?";
}

// -----------------------------------------------------------------------------
// Configuration de l'UART
// -----------------------------------------------------------------------------

void RadioReceiver::begin(int uartNumber, int receivePin, uint32_t timeoutMs,
                          RadioProtocol requestedProtocol)
{
    frameTimeoutMs       = timeoutMs;
    configuredUartNumber = uartNumber;
    configuredReceivePin = receivePin;
    serialPort           = (uartNumber == 2) ? &radioSerialPort2 : &radioSerialPort1;

    // La taille du tampon de reception doit etre fixee AVANT begin() : une fois
    // l'UART demarree, la demande est ignoree.
    serialPort->setRxBufferSize(256);

    applyProtocol(requestedProtocol);
}

void RadioReceiver::applyProtocol(RadioProtocol requestedProtocol)
{
    if (serialPort == nullptr) {
        return;
    }

    protocol = requestedProtocol;

    serialPort->end();

    // Le dernier parametre demande a l'UART d'inverser elle-meme le signal RX.
    // Le SBUS est electriquement inverse, l'iBUS ne l'est pas.
    if (protocol == RadioProtocol::Ibus) {
        serialPort->begin(115200, SERIAL_8N1, configuredReceivePin, -1, false);
    } else {
        serialPort->begin(100000, SERIAL_8E2, configuredReceivePin, -1, true);
    }

    // Repart d'un etat propre : les octets du reglage precedent n'ont plus de sens.
    frameByteCount = 0;
    while (serialPort->available() > 0) {
        serialPort->read();
    }

    LOG_INFO(logModule, "UART%d sur GPIO %d : %s",
             configuredUartNumber, configuredReceivePin,
             protocol == RadioProtocol::Ibus ? "iBUS, 115200 8N1 non inverse"
                                             : "SBUS, 100000 8E2 inverse");
}

bool RadioReceiver::detectProtocol(RadioProtocol preferredProtocol, uint32_t listenDurationMs)
{
    // Le protocole demande est essaye en premier : un reglage correct est ainsi
    // confirme sans jamais basculer sur un autre.
    const RadioProtocol candidates[2] = {
        preferredProtocol,
        (preferredProtocol == RadioProtocol::Sbus) ? RadioProtocol::Ibus : RadioProtocol::Sbus,
    };

    for (const RadioProtocol candidate : candidates) {
        applyProtocol(candidate);

        const uint32_t previousValidFrameCount = validFrameCount;
        const uint32_t listenStartMs = millis();

        while ((millis() - listenStartMs) < listenDurationMs) {
            readIncomingFrames();
            if (validFrameCount > previousValidFrameCount) {
                LOG_INFO(logModule, "Recepteur reconnu : %s", radioProtocolName(candidate));
                return true;
            }
            delay(2);
        }

        LOG_WARN(logModule, "Aucune trame %s valide (%lu octets bruts lus au total)",
                 radioProtocolName(candidate),
                 static_cast<unsigned long>(rawByteCount));
    }

    // Aucun protocole ne fonctionne : on revient au reglage demande.
    applyProtocol(preferredProtocol);
    return false;
}

void RadioReceiver::setFrameTimeoutMs(uint32_t timeoutMs)
{
    frameTimeoutMs = timeoutMs;
}

int RadioReceiver::expectedFrameLength() const
{
    return (protocol == RadioProtocol::Ibus) ? ibusFrameLength : sbusFrameLength;
}

// -----------------------------------------------------------------------------
// Lecture du flux
// -----------------------------------------------------------------------------

bool RadioReceiver::readIncomingFrames()
{
    if (serialPort == nullptr) {
        return false;
    }

    bool decodedNewFrame = false;

    while (serialPort->available() > 0) {
        const uint8_t  incomingByte = static_cast<uint8_t>(serialPort->read());
        const uint32_t nowMicroseconds = micros();

        // Trace brute, independante du decodage : elle prouve qu'un signal
        // arrive reellement sur la broche, meme si aucune trame n'est valide.
        ++rawByteCount;
        rawByteDump[rawByteDumpIndex] = incomingByte;
        rawByteDumpIndex = (rawByteDumpIndex + 1) % radioRawDumpSize;

        if (consumeByte(incomingByte, nowMicroseconds)) {
            decodedNewFrame = true;
        }
    }

    return decodedNewFrame;
}

bool RadioReceiver::consumeByte(uint8_t incomingByte, uint32_t nowMicroseconds)
{
    // Un silence prolonge marque la fin de la trame precedente : tout octet
    // recu ensuite est forcement le debut d'une nouvelle trame.
    if ((nowMicroseconds - lastByteTimeMicroseconds) > interFrameGapMicroseconds) {
        frameByteCount = 0;
    }
    lastByteTimeMicroseconds = nowMicroseconds;

    // Verification de l'entete, propre a chaque protocole. Tant qu'elle echoue,
    // aucun octet n'est accumule : le decodeur reste en attente de synchronisation.
    if (protocol == RadioProtocol::Ibus) {
        if (frameByteCount == 0 && incomingByte != ibusLengthByte) {
            return false;
        }
        if (frameByteCount == 1 && incomingByte != ibusCommandByte) {
            // Le premier octet etait fortuit : on repart de zero, en gardant la
            // possibilite que celui-ci soit un debut de trame.
            frameByteCount = (incomingByte == ibusLengthByte) ? 1 : 0;
            frameBuffer[0] = ibusLengthByte;
            return false;
        }
    } else {
        if (frameByteCount == 0 && incomingByte != sbusHeaderByte) {
            return false;
        }
    }

    frameBuffer[frameByteCount] = incomingByte;
    ++frameByteCount;

    if (frameByteCount < expectedFrameLength()) {
        return false;
    }

    frameByteCount = 0;

    const bool wasDecoded = (protocol == RadioProtocol::Ibus) ? decodeIbusFrame()
                                                              : decodeSbusFrame();
    if (!wasDecoded) {
        ++decodingErrorCount;
    }
    return wasDecoded;
}

// -----------------------------------------------------------------------------
// Decodage SBUS
// -----------------------------------------------------------------------------

bool RadioReceiver::decodeSbusFrame()
{
    // Les recepteurs n'utilisent pas tous le meme octet de fin : la plupart
    // emettent 0x00, certains y placent des bits de telemetrie.
    const uint8_t footerByte = frameBuffer[24];
    const bool isFooterValid = (footerByte == 0x00) || ((footerByte & 0x0F) == 0x04);

    if (!isFooterValid) {
        return false;
    }

    // Les 16 voies occupent 11 bits chacune, contigues, sans alignement sur les
    // octets. On les extrait avec un registre a decalage.
    uint32_t bitAccumulator = 0;
    int      availableBits  = 0;
    int      channelIndex   = 0;

    for (int byteIndex = 1; byteIndex <= 22; ++byteIndex) {
        bitAccumulator |= static_cast<uint32_t>(frameBuffer[byteIndex]) << availableBits;
        availableBits  += 8;

        while (availableBits >= 11 && channelIndex < radioChannelCount) {
            channels[channelIndex] = static_cast<uint16_t>(bitAccumulator & 0x07FF);
            ++channelIndex;
            bitAccumulator >>= 11;
            availableBits   -= 11;
        }
    }

    const uint8_t flagsByte = frameBuffer[23];
    frameLostFlag = (flagsByte & 0x04) != 0;
    failsafeFlag  = (flagsByte & 0x08) != 0;

    if (frameLostFlag) {
        ++frameLostCount;
    }

    registerValidFrame();
    return true;
}

// -----------------------------------------------------------------------------
// Decodage iBUS
// -----------------------------------------------------------------------------

bool RadioReceiver::decodeIbusFrame()
{
    // Somme de controle : 0xFFFF moins la somme des 30 premiers octets.
    uint16_t computedChecksum = 0xFFFF;
    for (int byteIndex = 0; byteIndex < ibusFrameLength - 2; ++byteIndex) {
        computedChecksum -= frameBuffer[byteIndex];
    }

    const uint16_t receivedChecksum =
        static_cast<uint16_t>(frameBuffer[30]) |
        (static_cast<uint16_t>(frameBuffer[31]) << 8);

    if (computedChecksum != receivedChecksum) {
        return false;
    }

    // 14 voies de 16 bits petit-boutiste, exprimees directement en microsecondes.
    for (int channelIndex = 0; channelIndex < ibusChannelCount; ++channelIndex) {
        const int byteIndex = 2 + channelIndex * 2;
        channels[channelIndex] =
            static_cast<uint16_t>(frameBuffer[byteIndex]) |
            (static_cast<uint16_t>(frameBuffer[byteIndex + 1]) << 8);
    }

    // Les voies 15 et 16 n'existent pas en iBUS : elles restent a zero.
    for (int channelIndex = ibusChannelCount; channelIndex < radioChannelCount; ++channelIndex) {
        channels[channelIndex] = 0;
    }

    // L'iBUS ne transporte aucun indicateur de perte : seul le chien de garde
    // temporel detecte une coupure de liaison.
    frameLostFlag = false;
    failsafeFlag  = false;

    registerValidFrame();
    return true;
}

// -----------------------------------------------------------------------------
// Etat
// -----------------------------------------------------------------------------

void RadioReceiver::registerValidFrame()
{
    const uint32_t nowMs = millis();

    if (hasReceivedAnyFrame) {
        const uint32_t intervalMs = nowMs - lastValidFrameTimeMs;
        if (intervalMs > 0) {
            const float instantRateHz = 1000.0f / static_cast<float>(intervalMs);
            measuredFrameRateHz += frameRateSmoothingFactor * (instantRateHz - measuredFrameRateHz);
        }
    } else {
        hasReceivedAnyFrame = true;
        LOG_INFO(logModule, "Premiere trame %s valide recue", radioProtocolName(protocol));
    }

    lastValidFrameTimeMs = nowMs;
    ++validFrameCount;
}

RadioStatus RadioReceiver::status() const
{
    const uint32_t nowMs = millis();

    RadioStatus currentStatus{};
    currentStatus.protocol   = protocol;
    currentStatus.frameAgeMs = hasReceivedAnyFrame ? (nowMs - lastValidFrameTimeMs) : UINT32_MAX;
    currentStatus.isConnected = hasReceivedAnyFrame && (currentStatus.frameAgeMs <= frameTimeoutMs);
    currentStatus.isFailsafeActive     = failsafeFlag;
    currentStatus.isFrameLost          = frameLostFlag;
    currentStatus.lastValidFrameTimeMs = lastValidFrameTimeMs;
    currentStatus.frameRateHz          = currentStatus.isConnected ? measuredFrameRateHz : 0.0f;
    currentStatus.validFrameCount      = validFrameCount;
    currentStatus.decodingErrorCount   = decodingErrorCount;
    currentStatus.frameLostCount       = frameLostCount;
    currentStatus.rawByteCount         = rawByteCount;

    return currentStatus;
}

uint16_t RadioReceiver::rawChannel(int channelIndex) const
{
    if (channelIndex < 0 || channelIndex >= radioChannelCount) {
        return 0;
    }
    return channels[channelIndex];
}

void RadioReceiver::copyRawChannels(uint16_t* destination) const
{
    memcpy(destination, channels, sizeof(channels));
}

int RadioReceiver::copyRawByteDump(uint8_t* destination) const
{
    // Restitue les octets du plus ancien au plus recent.
    for (int offset = 0; offset < radioRawDumpSize; ++offset) {
        destination[offset] = rawByteDump[(rawByteDumpIndex + offset) % radioRawDumpSize];
    }
    return radioRawDumpSize;
}

// -----------------------------------------------------------------------------
// Conditionnement des voies
// -----------------------------------------------------------------------------

float normalizeRadioChannel(uint16_t rawValue, const AxisConfiguration& axis)
{
    const float raw    = static_cast<float>(rawValue);
    const float center = static_cast<float>(axis.rawCenter);

    // Etape 1 : calibration min / centre / max vers -1..+1.
    // Les deux demi-courses sont mises a l'echelle independamment, ce qui gere
    // les manches dont le neutre n'est pas exactement au milieu.
    float normalized = 0.0f;

    if (raw > center) {
        const float upperSpan = static_cast<float>(axis.rawMaximum) - center;
        normalized = (upperSpan > 1.0f) ? (raw - center) / upperSpan : 0.0f;
    } else if (raw < center) {
        const float lowerSpan = center - static_cast<float>(axis.rawMinimum);
        normalized = (lowerSpan > 1.0f) ? (raw - center) / lowerSpan : 0.0f;
    }

    normalized = constrain(normalized, -1.0f, 1.0f);

    if (axis.inverted) {
        normalized = -normalized;
    }

    // Etape 2 : zone morte, avec remise a l'echelle pour conserver la pleine course.
    const float deadband = constrain(axis.deadband, 0.0f, 0.5f);
    const float magnitude = fabsf(normalized);

    if (magnitude < deadband) {
        normalized = 0.0f;
    } else {
        const float sign = (normalized < 0.0f) ? -1.0f : 1.0f;
        normalized = sign * (magnitude - deadband) / (1.0f - deadband);
    }

    // Etape 3 : courbe exponentielle (0 = lineaire, 1 = cubique).
    const float expo = constrain(axis.expo, 0.0f, 1.0f);
    normalized = (1.0f - expo) * normalized + expo * normalized * normalized * normalized;

    // Etape 4 : gain utilisateur.
    normalized *= constrain(axis.gain, 0.0f, 1.0f);

    return constrain(normalized, -1.0f, 1.0f);
}
