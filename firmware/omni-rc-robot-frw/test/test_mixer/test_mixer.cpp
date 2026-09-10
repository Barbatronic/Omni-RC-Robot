/**
 * @file test_mixer.cpp
 *
 * @brief Tests unitaires du mixer omnidirectionnel, executes sur PC.
 *
 * Ces tests couvrent les criteres de validation mathematiques de la
 * specification, sans aucun materiel :
 *
 *   Test 2  neutre                 -> toutes les sorties nulles
 *   Test 3  translation avant      -> combinaison symetrique attendue
 *   Test 4  translation laterale   -> aucune composante longitudinale parasite
 *   Test 5  rotation               -> rotation sur place, sans translation
 *   Test 6  combinaison            -> translation et rotation simultanees
 *   Test 7  saturation             -> aucune sortie hors de la plage autorisee
 *
 * Lancement :  pio test -e native_mixer_test
 */

#include <unity.h>

#include <cmath>

#include "omni_drive_mixer.h"

namespace
{
    constexpr float tolerance = 0.001f;
    constexpr float degreesToRadians = 3.14159265358979323846f / 180.0f;

    /// Geometrie de reference : quatre roues symetriques a 60 degres.
    WheelConfiguration makeTestWheels(WheelConfiguration (&wheels)[wheelCount], float angleDeg)
    {
        applySymmetricWheelGeometry(wheels, angleDeg);
        for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
            wheels[wheelIndex].outputGain = 1.0f;
            wheels[wheelIndex].inverted   = false;
            wheels[wheelIndex].enabled    = true;
        }
        return wheels[0];
    }

    /**
     * @brief Recompose le mouvement du chassis a partir des commandes de roues.
     *
     * Somme les vecteurs de poussee de chaque roue. Permet de verifier qu'une
     * commande de translation ne produit pas de mouvement parasite.
     */
    void computeResultingMotion(const WheelOutputs& outputs,
                                const WheelConfiguration (&wheels)[wheelCount],
                                float& resultX, float& resultY)
    {
        resultX = 0.0f;
        resultY = 0.0f;
        for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
            const float angleRadians = wheels[wheelIndex].driveAngleDeg * degreesToRadians;
            resultX += outputs.values[wheelIndex] * cosf(angleRadians);
            resultY += outputs.values[wheelIndex] * sinf(angleRadians);
        }
    }
}

// --- Test 2 : manches au neutre ---------------------------------------------
void test_neutral_command_stops_all_motors()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    const WheelOutputs outputs = computeWheelCommands(DriveCommand{}, wheels, 1.0f);

    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0f, outputs.values[wheelIndex]);
    }
}

// --- Test 3 : translation avant ---------------------------------------------
void test_forward_translation_is_symmetric()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    DriveCommand command;
    command.translationY = 1.0f;
    const WheelOutputs outputs = computeWheelCommands(command, wheels, 1.0f);

    // Les roues gauches (M0, M3) et droites (M1, M2) sont symetriques.
    TEST_ASSERT_FLOAT_WITHIN(tolerance, -outputs.values[1], outputs.values[0]);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, -outputs.values[2], outputs.values[3]);

    // Le mouvement resultant est purement longitudinal, vers l'avant.
    float resultX = 0.0f, resultY = 0.0f;
    computeResultingMotion(outputs, wheels, resultX, resultY);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0f, resultX);
    TEST_ASSERT_TRUE(resultY > 0.5f);
}

// --- Test 4 : translation laterale ------------------------------------------
void test_lateral_translation_has_no_longitudinal_component()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    DriveCommand command;
    command.translationX = 1.0f;
    const WheelOutputs outputs = computeWheelCommands(command, wheels, 1.0f);

    float resultX = 0.0f, resultY = 0.0f;
    computeResultingMotion(outputs, wheels, resultX, resultY);

    TEST_ASSERT_TRUE(resultX > 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0f, resultY);
}

// --- Test 5 : rotation sur place --------------------------------------------
void test_rotation_produces_no_translation()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    DriveCommand command;
    command.rotation = 1.0f;
    const WheelOutputs outputs = computeWheelCommands(command, wheels, 1.0f);

    // Les quatre roues tournent dans le meme sens...
    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        TEST_ASSERT_FLOAT_WITHIN(tolerance, 1.0f, outputs.values[wheelIndex]);
    }

    // ... et la somme des poussees ne deplace pas le robot.
    float resultX = 0.0f, resultY = 0.0f;
    computeResultingMotion(outputs, wheels, resultX, resultY);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0f, resultX);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0f, resultY);
}

// --- Test 6 : translation et rotation simultanees ---------------------------
void test_combined_translation_and_rotation()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    DriveCommand command;
    command.translationY = 0.5f;
    command.rotation     = 0.5f;
    const WheelOutputs outputs = computeWheelCommands(command, wheels, 1.0f);

    // Les quatre commandes doivent differer : la rotation brise la symetrie.
    bool hasDistinctValues = false;
    for (int wheelIndex = 1; wheelIndex < wheelCount; ++wheelIndex) {
        if (fabsf(outputs.values[wheelIndex] - outputs.values[0]) > tolerance) {
            hasDistinctValues = true;
        }
    }
    TEST_ASSERT_TRUE(hasDistinctValues);

    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        TEST_ASSERT_TRUE(fabsf(outputs.values[wheelIndex]) <= 1.0f + tolerance);
    }
}

// --- Test 7 : saturation ------------------------------------------------------
void test_maximum_command_never_exceeds_range()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    // Balayage complet de l'espace des commandes, aux valeurs extremes.
    const float sweepValues[] = { -1.0f, -0.5f, 0.0f, 0.5f, 1.0f };

    for (float vx : sweepValues) {
        for (float vy : sweepValues) {
            for (float omega : sweepValues) {
                DriveCommand command;
                command.translationX = vx;
                command.translationY = vy;
                command.rotation     = omega;

                const WheelOutputs outputs = computeWheelCommands(command, wheels, 1.0f);
                for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
                    TEST_ASSERT_TRUE(outputs.values[wheelIndex] <= 1.0f + tolerance);
                    TEST_ASSERT_TRUE(outputs.values[wheelIndex] >= -1.0f - tolerance);
                }
            }
        }
    }
}

// --- Limitation de puissance ---------------------------------------------------
void test_maximum_output_limit_is_applied()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    DriveCommand command;
    command.translationX = 1.0f;
    command.translationY = 1.0f;
    command.rotation     = 1.0f;

    const WheelOutputs outputs = computeWheelCommands(command, wheels, 0.25f);

    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        TEST_ASSERT_TRUE(fabsf(outputs.values[wheelIndex]) <= 0.25f + tolerance);
    }
}

// --- Reglages par roue ---------------------------------------------------------
void test_disabled_and_inverted_wheels()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    wheels[0].enabled  = false;
    wheels[1].inverted = true;

    DriveCommand command;
    command.rotation = 1.0f;
    const WheelOutputs outputs = computeWheelCommands(command, wheels, 1.0f);

    TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0f, outputs.values[0]);   // desactivee
    TEST_ASSERT_FLOAT_WITHIN(tolerance, -1.0f, outputs.values[1]);  // inversee
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 1.0f, outputs.values[2]);
}

// --- Compensation du seuil de demarrage ---------------------------------------
void test_startup_threshold_lifts_small_commands()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    DriveCommand command;
    command.rotation = 0.05f;   // commande volontairement tres faible

    // Sans compensation : la sortie reste sous le seuil de demarrage du moteur.
    const WheelOutputs withoutThreshold = computeWheelCommands(command, wheels, 1.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.05f, withoutThreshold.values[0]);

    // Avec compensation : la roue tourne des la plus petite commande non nulle.
    const WheelOutputs withThreshold = computeWheelCommands(command, wheels, 1.0f, 0.20f);
    TEST_ASSERT_TRUE(withThreshold.values[0] >= 0.20f);
}

void test_startup_threshold_preserves_full_scale_and_limit()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    DriveCommand command;
    command.rotation = 1.0f;

    // A pleine commande, la sortie vaut exactement le plafond : la compensation
    // comprime la plage utile, elle ne deborde jamais la limite de puissance.
    const WheelOutputs atFullScale = computeWheelCommands(command, wheels, 0.50f, 0.20f);
    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.50f, atFullScale.values[wheelIndex]);
    }
}

void test_startup_threshold_keeps_zero_at_zero()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    // Manches au neutre : aucune roue ne doit etre remontee au seuil.
    const WheelOutputs outputs = computeWheelCommands(DriveCommand{}, wheels, 1.0f, 0.25f);
    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.0f, outputs.values[wheelIndex]);
    }
}

void test_startup_threshold_preserves_sign()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    DriveCommand command;
    command.rotation = -0.05f;
    const WheelOutputs outputs = computeWheelCommands(command, wheels, 1.0f, 0.20f);

    // Le sens de rotation doit etre conserve, seule l'amplitude est remontee.
    TEST_ASSERT_TRUE(outputs.values[0] <= -0.20f);
}

void test_startup_threshold_above_limit_is_clamped()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    DriveCommand command;
    command.rotation = 0.3f;

    // Minimum superieur au plafond : la sortie s'aligne sur le plafond et ne le
    // depasse pas, plutot que de produire une valeur incoherente.
    const WheelOutputs outputs = computeWheelCommands(command, wheels, 0.20f, 0.90f);
    for (int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex) {
        TEST_ASSERT_TRUE(fabsf(outputs.values[wheelIndex]) <= 0.20f + tolerance);
    }
}

// --- Rattrapage de la perte geometrique ----------------------------------------
void test_translation_normalization_reaches_full_scale()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    // Marche avant plein manche : sans rattrapage la roue la plus chargee
    // n'atteint que la moitie de l'echelle.
    DriveCommand forward;
    forward.translationY = 1.0f;

    const WheelOutputs plain = computeWheelCommands(forward, wheels, 1.0f, 0.0f, false);
    float largestPlain = 0.0f;
    for (int i = 0; i < wheelCount; ++i) {
        largestPlain = fmaxf(largestPlain, fabsf(plain.values[i]));
    }
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 0.5f, largestPlain);

    // Avec rattrapage, la roue la plus chargee atteint exactement la pleine echelle.
    const WheelOutputs scaled = computeWheelCommands(forward, wheels, 1.0f, 0.0f, true);
    float largestScaled = 0.0f;
    for (int i = 0; i < wheelCount; ++i) {
        largestScaled = fmaxf(largestScaled, fabsf(scaled.values[i]));
    }
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 1.0f, largestScaled);
}

void test_translation_normalization_is_direction_only()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    // Point critique de securite : une commande faible doit rester faible.
    // Le facteur est calcule sur la DIRECTION, pas sur l'amplitude.
    DriveCommand small;
    small.translationY = 0.1f;
    const WheelOutputs scaledSmall = computeWheelCommands(small, wheels, 1.0f, 0.0f, true);

    DriveCommand full;
    full.translationY = 1.0f;
    const WheelOutputs scaledFull = computeWheelCommands(full, wheels, 1.0f, 0.0f, true);

    // Un dixieme de manche doit donner un dixieme de sortie, pas la pleine echelle.
    for (int i = 0; i < wheelCount; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(tolerance, scaledFull.values[i] * 0.1f, scaledSmall.values[i]);
    }
}

void test_translation_normalization_all_directions_bounded()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    // Toutes directions, rotation comprise : rien ne doit sortir de la plage.
    const float sweep[] = { -1.0f, -0.5f, 0.0f, 0.5f, 1.0f };
    for (float vx : sweep) {
        for (float vy : sweep) {
            for (float omega : sweep) {
                DriveCommand command;
                command.translationX = vx;
                command.translationY = vy;
                command.rotation     = omega;

                const WheelOutputs outputs =
                    computeWheelCommands(command, wheels, 1.0f, 0.0f, true);
                for (int i = 0; i < wheelCount; ++i) {
                    TEST_ASSERT_TRUE(fabsf(outputs.values[i]) <= 1.0f + tolerance);
                }
            }
        }
    }
}

void test_translation_normalization_leaves_rotation_alone()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 60.0f);

    // La rotation atteint deja la pleine echelle : le rattrapage ne doit rien changer.
    DriveCommand rotation;
    rotation.rotation = 1.0f;

    const WheelOutputs plain  = computeWheelCommands(rotation, wheels, 1.0f, 0.0f, false);
    const WheelOutputs scaled = computeWheelCommands(rotation, wheels, 1.0f, 0.0f, true);
    for (int i = 0; i < wheelCount; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(tolerance, plain.values[i], scaled.values[i]);
    }
}

// --- Geometrie 45 degres : equivalence avec un X-drive classique ---------------
void test_forty_five_degree_preset_matches_x_drive()
{
    WheelConfiguration wheels[wheelCount];
    makeTestWheels(wheels, 45.0f);

    TEST_ASSERT_FLOAT_WITHIN(tolerance, 225.0f, wheels[0].driveAngleDeg);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 135.0f, wheels[1].driveAngleDeg);
    TEST_ASSERT_FLOAT_WITHIN(tolerance,  45.0f, wheels[2].driveAngleDeg);
    TEST_ASSERT_FLOAT_WITHIN(tolerance, 315.0f, wheels[3].driveAngleDeg);
}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(test_neutral_command_stops_all_motors);
    RUN_TEST(test_forward_translation_is_symmetric);
    RUN_TEST(test_lateral_translation_has_no_longitudinal_component);
    RUN_TEST(test_rotation_produces_no_translation);
    RUN_TEST(test_combined_translation_and_rotation);
    RUN_TEST(test_maximum_command_never_exceeds_range);
    RUN_TEST(test_maximum_output_limit_is_applied);
    RUN_TEST(test_disabled_and_inverted_wheels);
    RUN_TEST(test_startup_threshold_lifts_small_commands);
    RUN_TEST(test_startup_threshold_preserves_full_scale_and_limit);
    RUN_TEST(test_startup_threshold_keeps_zero_at_zero);
    RUN_TEST(test_startup_threshold_preserves_sign);
    RUN_TEST(test_startup_threshold_above_limit_is_clamped);
    RUN_TEST(test_translation_normalization_reaches_full_scale);
    RUN_TEST(test_translation_normalization_is_direction_only);
    RUN_TEST(test_translation_normalization_all_directions_bounded);
    RUN_TEST(test_translation_normalization_leaves_rotation_alone);
    RUN_TEST(test_forty_five_degree_preset_matches_x_drive);
    return UNITY_END();
}
