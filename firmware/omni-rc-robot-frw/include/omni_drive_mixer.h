#pragma once

/**
 * @file omni_drive_mixer.h
 *
 * @brief Transformation des consignes de pilotage en commandes des quatre roues.
 *
 * Entrees  : vx, vy, omega (chacun entre -1 et +1) et la geometrie des roues.
 * Sorties  : quatre commandes moteur normalisees entre -1 et +1.
 * Dialogue avec : RobotController uniquement.
 *
 * Ce module ne contient QUE des mathematiques : aucune dependance a Arduino ni
 * a un peripherique. Il est donc compilable et testable sur PC, sans carte
 * (voir test/test_mixer et l'environnement PlatformIO native_mixer_test).
 *
 * ---------------------------------------------------------------------------
 * Convention de coordonnees (robot vu de dessus)
 * ---------------------------------------------------------------------------
 *
 *                          +Y  (avant)
 *                           ^
 *                           |
 *              M0           |            M1
 *                           |
 *          -X <-------------O-------------> +X  (droite)
 *                           |
 *              M3           |            M2
 *                           |
 *                           v
 *                          -Y  (arriere)
 *
 *   vx    > 0 : le robot se deplace vers la DROITE
 *   vy    > 0 : le robot se deplace vers l'AVANT
 *   omega > 0 : le robot tourne dans le sens ANTIHORAIRE
 *
 * Les angles sont mesures en degres, dans le sens antihoraire, depuis l'axe +X.
 * Ainsi 0 degre pointe a droite et 90 degres pointe vers l'avant.
 *
 * ---------------------------------------------------------------------------
 * Description mathematique du mixage
 * ---------------------------------------------------------------------------
 *
 * Pour chaque roue i, driveAngleDeg est l'angle theta_i de sa direction motrice :
 * la direction dans laquelle son point de contact pousse le chassis lorsque le
 * moteur tourne dans le sens positif.
 *
 * La commande de la roue est la projection du vecteur vitesse demande sur cette
 * direction motrice, augmentee de la contribution de rotation :
 *
 *     m_i = vx * cos(theta_i) + vy * sin(theta_i) + omega * rotationGain_i
 *
 * Cette formulation est volontairement parametrique : elle ne suppose aucune
 * geometrie particuliere (Mecanum, X-drive, 60 degres). Adapter le robot a sa
 * mecanique reelle ne demande que d'ajuster les angles et les gains, jamais de
 * reecrire l'algorithme.
 *
 * Une compensation de seuil de demarrage peut enfin etre appliquee : la plage
 * utile de chaque moteur est comprimee de [0, maximumOutput] vers
 * [minimumOutput, maximumOutput], de sorte qu'une commande faible fasse
 * reellement tourner la roue au lieu de laisser le moteur bloque. La limite de
 * puissance n'est jamais depassee.
 *
 * Si la plus grande commande depasse 1 en valeur absolue, les quatre commandes
 * sont divisees par ce maximum. La direction globale demandee est ainsi
 * preservee, au prix d'une reduction homogene de la vitesse, plutot que de
 * saturer une seule roue et de devier de la trajectoire.
 */

#include "robot_config.h"

/**
 * @brief Consigne de pilotage demandee au robot.
 */
struct DriveCommand
{
    float translationX = 0.0f;  ///< vx, positif vers la droite
    float translationY = 0.0f;  ///< vy, positif vers l'avant
    float rotation     = 0.0f;  ///< omega, positif en antihoraire
};

/**
 * @brief Resultat du mixage pour les quatre roues.
 */
struct WheelOutputs
{
    float values[wheelCount] = { 0.0f, 0.0f, 0.0f, 0.0f };
};

/**
 * @brief Calcule les commandes des quatre roues a partir d'une consigne.
 *
 * Applique la projection geometrique, la normalisation globale, les gains
 * individuels de roue et le plafond de puissance.
 *
 * @param command Consigne vx / vy / omega, chaque composante entre -1 et +1.
 * @param wheels Geometrie et reglages des quatre roues.
 * @param maximumOutput Plafond applique a chaque sortie, entre 0 et 1.
 * @param minimumOutput Sortie minimale d'un moteur en mouvement, pour franchir
 *        son seuil de demarrage. 0 desactive la compensation.
 * @param normalizeTranslation true pour exploiter toute la capacite moteur quelle
 *        que soit la direction de translation demandee.
 * @return Commandes des quatre roues, garanties dans -maximumOutput..+maximumOutput.
 */
WheelOutputs computeWheelCommands(const DriveCommand& command,
                                  const WheelConfiguration (&wheels)[wheelCount],
                                  float maximumOutput,
                                  float minimumOutput = 0.0f,
                                  bool normalizeTranslation = false);

/**
 * @brief Renseigne des angles de roues symetriques pour une geometrie donnee.
 *
 * @c geometryAngleDeg est l'angle entre la direction motrice d'une roue et
 * l'axe longitudinal du robot : 45 degres donne un X-drive classique, 60 degres
 * correspond a la geometrie annoncee de ce robot.
 *
 * Les gains de rotation sont mis a +1 sur les quatre roues, ce qui produit une
 * rotation antihoraire pour omega positif avec cette convention d'angles.
 *
 * @param wheels Roues a configurer.
 * @param geometryAngleDeg Angle de la geometrie, en degres.
 */
void applySymmetricWheelGeometry(WheelConfiguration (&wheels)[wheelCount],
                                 float geometryAngleDeg);
