# Robot Agricole V3

## Introduction
Ce projet implémente un robot agricole autonome basé sur ESP32, capable de naviguer en ligne droite, d'éviter des obstacles, de détecter et ramasser des objets, et de fonctionner de manière multitâche grâce à FreeRTOS. Il est conçu pour des applications de robotique agricole, de recherche ou d'enseignement.

## Objectifs
- Navigation autonome avec correction gyroscopique
- Détection et ramassage d'objets (jetons)
- Évitement d'obstacles par capteur ultrason
- Supervision par LED et logs série
- Architecture logicielle modulaire et multitâche

## Architecture matérielle
- **Microcontrôleur** : ESP32 (dual core)
- **Gyroscope/Accéléromètre** : MPU6050
- **Capteurs IR** : détection d'objets
- **Capteur ultrason** : HC-SR04 pour l'évitement d'obstacles
- **Servomoteur** : mécanisme de ramassage
- **LED** : supervision d'état
- **Moteurs DC** : pilotés par pont en H

## Fonctionnalités principales
- Navigation en ligne droite avec correction gyroscopique (filtre de Kalman)
- Détection d'objets par capteurs IR
- Ramassage automatique via servomoteur
- Évitement d'obstacles par manœuvre spéciale (ultrason)
- Supervision d'état par LED et logs série
- Gestion multitâche avec FreeRTOS (tâches séparées pour chaque fonction)

## Organisation logicielle
Le code est organisé autour de plusieurs tâches FreeRTOS :
- **TaskSupervision** : gestion de la LED d'état
- **TaskGyroControl** : correction de trajectoire par gyroscope
- **TaskDetectJeton** : détection d'objets (IR)
- **TaskRamassageJeton** : séquence de ramassage
- **TaskUltrasonicDetection** : détection d'obstacles (ultrason)
- **TaskManeuver** : manœuvre d'évitement spéciale

Chaque tâche fonctionne de façon indépendante et communique via des sémaphores et notifications.

## Dépendances et installation
Ce projet utilise [PlatformIO](https://platformio.org/) et le framework Arduino pour ESP32.

**Fichier `platformio.ini` :**
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    adafruit/Adafruit MPU6050@^2.2.6
    madhephaestus/ESP32Servo@^3.0.6
    teckel12/NewPing@^1.9.7
```

### Installation
1. Installer [PlatformIO](https://platformio.org/install/ide?install=vscode)
2. Cloner ce dépôt :
   ```bash
   git clone <url-du-repo>
   cd Robot-agricol-V3
   ```
3. Ouvrir le dossier dans VSCode avec PlatformIO
4. Connecter l'ESP32 via USB
5. Compiler et téléverser avec PlatformIO (`Upload`)

## Structure du projet
```
├── src/
│   └── main.cpp           # Code principal du robot
├── lib/
│   └── KalmanFilter/      # Filtre de Kalman personnalisé
├── include/               # Fichiers d'en-tête (headers)
├── test/                  # Tests unitaires (PlatformIO)
├── platformio.ini         # Configuration PlatformIO
└── README.md              # Ce fichier
```

## Bibliothèque KalmanFilter
Le projet inclut une implémentation personnalisée d'un filtre de Kalman (lib/KalmanFilter) pour la fusion de données gyroscopiques et l'amélioration de la stabilité de la trajectoire.
- `KF` : classe principale du filtre
- `predict(dt, gyro_rate)` : prédiction d'état
- `update(measValue, measVariance)` : correction par mesure

## Utilisation et personnalisation
- **Logs** : Les logs sont activés par défaut (`#define ENABLE_LOGS` dans `main.cpp`).
- **Calibration** : La calibration du gyroscope s'effectue automatiquement au démarrage.
- **Paramètres** : Les constantes (vitesses, seuils, pins) sont configurables en début de `main.cpp`.
- **Ajout de capteurs** : Ajouter le code dans les tâches correspondantes.

## Compilation et téléversement
- Utiliser les boutons `Build` et `Upload` de PlatformIO dans VSCode.
- Le port série par défaut est 115200 bauds.

## Conseils
- Vérifier le câblage des moteurs et capteurs avant la mise sous tension.
- Adapter les pins dans `main.cpp` selon votre montage.
- Pour un usage en extérieur, protéger l'électronique contre l'humidité.

## Crédits
- Développement : BARRA, Dorus Tsitera
- Année : 2025

## Licence
Ce projet est sous licence MIT. Voir le fichier LICENSE si présent. 