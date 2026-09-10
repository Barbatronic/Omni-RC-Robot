#pragma once

/**
 * @file wifi_secrets.example.h
 *
 * @brief Modele du fichier d'identifiants Wi-Fi.
 *
 * Ce fichier-ci est suivi par Git et ne contient AUCUN secret.
 *
 * Pour connecter le robot a un reseau existant :
 *
 *   1. copier ce fichier :
 *        cp include/wifi_secrets.example.h include/wifi_secrets.h
 *   2. renseigner le SSID et le mot de passe reels dans include/wifi_secrets.h
 *   3. ne jamais commiter include/wifi_secrets.h : il est exclu par .gitignore
 *
 * Si include/wifi_secrets.h est absent, le projet compile quand meme : le robot
 * demarre alors en point d'acces et le reseau d'infrastructure se configure
 * depuis l'interface Web.
 */

// SSID du reseau Wi-Fi existant a rejoindre (mode Station).
#define WIFI_STATION_SSID     ""

// Mot de passe de ce reseau.
#define WIFI_STATION_PASSWORD ""

// Mot de passe du point d'acces cree par le robot (mode AP).
// Au moins 8 caracteres, ou vide pour un reseau ouvert.
#define WIFI_ACCESS_POINT_PASSWORD ""

// Mode reseau au premier demarrage : 0 = point d'acces, 1 = station.
#define WIFI_DEFAULT_MODE_IS_STATION 0
