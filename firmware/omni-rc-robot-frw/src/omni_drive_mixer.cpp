/**
 * @file omni_drive_mixer.cpp
 *
 * @brief Implementation du mixage omnidirectionnel a quatre roues.
 *
 * Voir omni_drive_mixer.h pour la convention de coordonnees et la description
 * mathematique complete.
 */

#include "omni_drive_mixer.h"

#include <cmath>

namespace
{
    constexpr float degreesToRadians = 3.14159265358979323846f / 180.0f;

    /// En deca de cette commande, la roue est consideree a l'arret.
    constexpr float negligibleCommand = 0.001f;

    float clampToUnitRange(float value, float limit)
    {
        if (value > limit) {
            return limit;
        }
        if (value < -limit) {
            return -limit;
        }
        return value;
    }

    /**
     * @brief Remonte une commande non nulle au-dessus du seuil de demarrage.
     *
     * La plage utile est comprimee de [0, outputLimit] vers
     * [minimumOutput, outputLimit] : la pleine echelle reste identique, mais la
     * plus petite commande non nulle produit deja minimumOutput. La limite de
     * puissance configuree n'est donc jamais depassee.
     */
    float applyStartupThreshold(float value, float outputLimit, float minimumOutput)
    {
        const float magnitude = fabsf(value);

        if (magnitude < negligibleCommand || minimumOutput <= 0.0f || outputLimit <= 0.0f) {
            return value;
        }

        // Un minimum superieur au plafond n'aurait pas de sens : on s'aligne alors
        // sur le plafond, ce qui rend la commande tout ou rien.
        const float effectiveMinimum = (minimumOutput < outputLimit) ? minimumOutput : outputLimit;
        const float remapped = effectiveMinimum
                             + (outputLimit - effectiveMinimum) * (magnitude / outputLimit);

        return (value < 0.0f) ? -remapped : remapped;
    }
}

/**
 * @brief Facteur permettant a une direction de translation d'atteindre la pleine echelle.
 *
 * Le facteur est calcule sur un vecteur de translation UNITAIRE de meme
 * direction que la consigne, jamais sur la consigne elle-meme. C'est ce qui
 * garantit qu'un petit mouvement de manche reste petit : seule la direction
 * intervient, l'amplitude est conservee telle quelle.
 *
 * @return Facteur superieur ou egal a 1, ou 1 si la translation est negligeable.
 */
static float translationFullScaleFactor(const DriveCommand& command,
                                        const WheelConfiguration (&wheels)[wheelCount])
{
    const float translationMagnitude =
        sqrtf(command.translationX * command.translationX
            + command.translationY * command.translationY);

    if (translationMagnitude < negligibleCommand) {
        return 1.0f;
    }

    const float unitX = command.translationX / translationMagnitude;
    const float unitY = command.translationY / translationMagnitude;

    float largestUnitProjection = 0.0f;
    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        const float angleRadians = wheels[wheelIndex].driveAngleDeg * degreesToRadians;
        const float projection = unitX * cosf(angleRadians) + unitY * sinf(angleRadians);
        const float magnitude = fabsf(projection);
        if (magnitude > largestUnitProjection) {
            largestUnitProjection = magnitude;
        }
    }

    if (largestUnitProjection < negligibleCommand) {
        return 1.0f;
    }
    return 1.0f / largestUnitProjection;
}

WheelOutputs computeWheelCommands(const DriveCommand& command,
                                  const WheelConfiguration (&wheels)[wheelCount],
                                  float maximumOutput,
                                  float minimumOutput,
                                  bool normalizeTranslation)
{
    WheelOutputs outputs;

    DriveCommand effectiveCommand = command;

    // Etape 0 : rattrapage de la perte due a la geometrie, si demande. La
    // rotation n'est pas touchee ; la saturation de l'etape 2 gere les
    // commandes combinees.
    if (normalizeTranslation) {
        const float scaleFactor = translationFullScaleFactor(command, wheels);
        effectiveCommand.translationX *= scaleFactor;
        effectiveCommand.translationY *= scaleFactor;
    }

    // Etape 1 : projection de la consigne sur la direction motrice de chaque roue.
    float largestMagnitude = 0.0f;

    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        const WheelConfiguration& wheel = wheels[wheelIndex];
        const float driveAngleRadians = wheel.driveAngleDeg * degreesToRadians;

        const float projectedCommand =
              effectiveCommand.translationX * cosf(driveAngleRadians)
            + effectiveCommand.translationY * sinf(driveAngleRadians)
            + effectiveCommand.rotation     * wheel.rotationGain;

        outputs.values[wheelIndex] = projectedCommand;

        const float magnitude = fabsf(projectedCommand);
        if (magnitude > largestMagnitude) {
            largestMagnitude = magnitude;
        }
    }

    // Etape 2 : normalisation globale. Si une roue depasse la pleine echelle, on
    // reduit les quatre dans le meme rapport pour conserver la direction demandee.
    if (largestMagnitude > 1.0f) {
        for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
            outputs.values[wheelIndex] /= largestMagnitude;
        }
    }

    // Etape 3 : reglages propres a chaque roue, puis plafond de puissance.
    const float outputLimit = clampToUnitRange(maximumOutput, 1.0f);

    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        const WheelConfiguration& wheel = wheels[wheelIndex];

        if (!wheel.enabled) {
            outputs.values[wheelIndex] = 0.0f;
            continue;
        }

        float wheelCommand = outputs.values[wheelIndex] * wheel.outputGain;

        if (wheel.inverted) {
            wheelCommand = -wheelCommand;
        }

        const float scaledCommand = clampToUnitRange(wheelCommand * outputLimit, outputLimit);
        outputs.values[wheelIndex] = applyStartupThreshold(scaledCommand, outputLimit, minimumOutput);
    }

    return outputs;
}

void applySymmetricWheelGeometry(WheelConfiguration (&wheels)[wheelCount],
                                 float geometryAngleDeg)
{
    // Directions motrices tangentielles, symetriques par rapport aux deux axes.
    // Pour 45 degres on retrouve exactement 225 / 135 / 45 / 315, soit un X-drive.
    const float angles[wheelCount] = {
        270.0f - geometryAngleDeg,  // M0 avant gauche
         90.0f + geometryAngleDeg,  // M1 avant droit
         90.0f - geometryAngleDeg,  // M2 arriere droit
        270.0f + geometryAngleDeg,  // M3 arriere gauche
    };

    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        wheels[wheelIndex].driveAngleDeg = angles[wheelIndex];
        wheels[wheelIndex].rotationGain  = 1.0f;
    }
}
