/**
 * @file main.cpp
 * @brief Robot agricole autonome avec capacités de navigation et d'évitement d'obstacles
 * 
 * Ce programme implémente un robot agricole autonome capable de :
 * - Naviguer en ligne droite avec correction gyroscopique
 * - Détecter et ramasser des objets
 * - Éviter les obstacles avec différentes stratégies
 * - Utiliser un système de supervision multi-tâches
 * 
 * Architecture matérielle :
 * - ESP32 (dual core)
 * - MPU6050 (gyroscope/accéléromètre)
 * - Capteurs IR pour détection d'objets
 * - Capteur ultrason HC-SR04
 * - Servomoteur pour le mécanisme de ramassage
 * - LED de supervision
 * 
 * @authors BARRA, Dorus Tsitera
 * @date Mai 2025
 * @version 3.0
 * 
 * @copyright Copyright (c) 2025
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include "KalmanFilter.h"
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include <NewPing.h>

// Configuration des logs
#define ENABLE_LOGS // Commenter cette ligne pour désactiver tous les logs

#ifdef ENABLE_LOGS
#define ENABLE_GYRO_LOGS      // Logs du gyroscope
#define ENABLE_DETECTION_LOGS // Logs de la détection
#define ENABLE_PICKUP_LOGS    // Logs du ramassage
#define ENABLE_SYSTEM_LOGS    // Logs système

#define LOG_GYRO(format, ...) \
    if (Serial)               \
    Serial.printf("[Gyro] " format "\n", ##__VA_ARGS__)
#define LOG_DETECTION(format, ...) \
    if (Serial)                    \
    Serial.printf("[Détection] " format "\n", ##__VA_ARGS__)
#define LOG_PICKUP(format, ...) \
    if (Serial)                 \
    Serial.printf("[Ramassage] " format "\n", ##__VA_ARGS__)
#define LOG_SYSTEM(format, ...) \
    if (Serial)                 \
    Serial.printf("[System] " format "\n", ##__VA_ARGS__)
#define LOG_ERROR(format, ...) \
    if (Serial)                \
    Serial.printf("[ERROR] " format "\n", ##__VA_ARGS__)
#else
#define LOG_GYRO(format, ...)
#define LOG_DETECTION(format, ...)
#define LOG_PICKUP(format, ...)
#define LOG_SYSTEM(format, ...)
#define LOG_ERROR(format, ...)
#endif

// Configuration LED de supervision
#define LED_SUPERVISION 2
#define BLINK_CALIBRATION 500    // Clignotement rapide pendant la calibration
#define BLINK_RUNNING 1000       // Clignotement normal en fonctionnement
#define BLINK_PICKING 200        // Clignotement très rapide pendant le ramassage
#define BLINK_ROTATION 300       // Clignotement pendant la rotation

// Configuration pour la manœuvre d'évitement
#define MANEUVER_TURN_TIME 500    // 2 secondes pour la rotation
#define MANEUVER_FORWARD_TIME 500  // 1 seconde pour avancer
#define MANEUVER_BACK_TIME 500     // 2 secondes pour reculer
#define MANEUVER_SPEED 75          // Vitesse pendant la manœuvre

// Variables pour le comptage des virages et manœuvres
volatile int nombreVirages = 0;
volatile bool maneuverInProgress = false;

// Handles pour les tâches
TaskHandle_t supervisionTaskHandle = NULL;  // Ajout du handle pour la tâche de supervision
TaskHandle_t gyroTaskHandle = NULL;
TaskHandle_t detectionTaskHandle = NULL;
TaskHandle_t pickupTaskHandle = NULL;
TaskHandle_t ultrasonicTaskHandle = NULL;
TaskHandle_t maneuverTaskHandle = NULL;  // Handle pour la tâche de manœuvre

// Mutex pour protéger l'accès aux vitesses des moteurs
SemaphoreHandle_t motorMutex = NULL;
SemaphoreHandle_t stateMutex = NULL;

// Variables pour la gestion de la rotation
volatile bool rotationEnCours = false;
volatile double yaw_initial = 0.0;
volatile double cap_reference = 0.0; // Cap de référence pour la ligne droite

// États du robot
enum RobotState
{
    RUNNING,        // Robot en marche normale
    STOPPING,       // En train de s'arrêter
    PICKING,        // En train de ramasser
    OBSTACLE,       // Détection d'obstacle
    ROTATION_GAUCHE // En rotation vers la gauche
};

// Variable d'état protégée
volatile RobotState currentState = RUNNING;

// Fonctions de gestion d'état thread-safe
void setRobotState(RobotState newState)
{
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        const char *stateStr;
        switch (newState)
        {
        case RUNNING:
            stateStr = "RUNNING";
            break;
        case STOPPING:
            stateStr = "STOPPING";
            break;
        case PICKING:
            stateStr = "PICKING";
            break;
        }
        LOG_SYSTEM("Changement d'état -> %s", stateStr);
        currentState = newState;
        xSemaphoreGive(stateMutex);
    }
}

RobotState getRobotState()
{
    RobotState state = RUNNING;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        state = currentState;
        xSemaphoreGive(stateMutex);
    }
    return state;
}

// Priorités des tâches
#define STRAIGHT_TASK_PRIORITY 1
#define PICKUP_TASK_PRIORITY 2

// Paramètres du servo
#define SERVO_PICKUP_POS 20
#define SERVO_INIT_POS 130
#define MAX_DISTANCE 200

// Période du timer pour la détection (5ms pour une détection précise)
#define DETECTION_TIMER_PERIOD 5

// Define MPU6050 address (can vary depending on your model)
#define MPU_ADDR 0x68
// Constantes pour le contrôle en ligne droite
const double KP = PI;        // Coefficient proportionnel
const int VITESSE_BASE = 75; // Vitesse de base des moteurs (0-255)
const int VITESSE_MAX = 255; // Vitesse maximale des moteurs
const int VITESSE_MIN = 50;  // Vitesse minimale des moteurs

// Création de l'objet Servo
Servo myservo;
// Broches pour les capteurs IR
const int IR_GAUCHE = 5;
const int IR_DROITE = 18;
const int SERVO_PIN = 19;

// Broches pour le capteur ultrason
const int TRIG_PIN = 13;
const int ECHO_PIN = 14;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // Création de l'objet sonar

Adafruit_MPU6050 mpu;

const int IN1 = 27, IN2 = 26;
const int IN3 = 25, IN4 = 32;
const int ENA1 = 16; // Changé pour GPIO 16 (PWM sûr)
const int ENB1 = 17; // Changé pour GPIO 17 (PWM sûr)
int vitesse = 100;

double gyroX, gyroY, gyroZ;                      // Angular velocities (rad/s)
double accelX, accelY, accelZ;                   // Accelerometer readings (g)
double angleX_accel, angleY_accel, angleZ_accel; // Rotation angles (degrees)
double angleX_gyro, angleY_gyro, angleZ_gyro;    // Rotation angles (degrees)
double pitch, roll;                              // Tilt angles (degrees)
double totalAccel;                               // Total acceleration (g)
double euler_roll;
double euler_pitch;
double mean = 0.0;
const double dev_stnd_accelleration = 0.1;  // Réduit pour moins faire confiance au modèle
double Measurement_Noise_Covariance = 0.01; // Réduit pour faire plus confiance aux mesures
double Measurement_Noise_Standard_deviation = sqrt(Measurement_Noise_Covariance);
double gyroX_cal = 0.0;
double gyroY_cal = 0.0;
double gyroZ_cal = 0.0;
const int CALIBRATION_SAMPLES = 2000; // Augmenté pour une meilleure calibration
double gyroX_offset = 0.0;
double gyroY_offset = 0.0;
double gyroZ_offset = 0.0;
double drift_correction = 0.0001; // Correction de dérive
double yaw = 0.0;                 // Angle yaw global pour le contrôle

double previousMillis = 0;
const int integrationInterval = 2; // 4ms = 250Hz
String valeur;

KF dk(0.00, 0.00, dev_stnd_accelleration *dev_stnd_accelleration);

// Variables pour le contrôle moteur
volatile int vitesseGauche = 0;
volatile int vitesseDroite = 0;

// Fonction de mise à jour des vitesses moteur avec protection mutex
void update_motor_speed(double correction)
{
    if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        vitesseGauche = VITESSE_BASE + correction;
        vitesseDroite = VITESSE_BASE - correction;

        // Limiter les vitesses
        vitesseGauche = constrain(vitesseGauche, VITESSE_MIN, VITESSE_MAX);
        vitesseDroite = constrain(vitesseDroite, VITESSE_MIN, VITESSE_MAX);

        xSemaphoreGive(motorMutex);
    }
}

// Fonction pour appliquer les vitesses aux moteurs
void avancer()
{
    if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        // Avant gauche
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA1, vitesseDroite);

        // Avant droite
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB1, vitesseGauche);

        xSemaphoreGive(motorMutex);
    }
}

void tournerDroite()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void arreter()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void reculer()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void tournerGauche()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

// Fonction de rotation précise à un angle cible
bool tournerGaucheAngle(float angleCible)
{
    // Mesurer l'angle initial
    float angleInitial = yaw;
    float angleFinal = angleInitial - 90.0; // On veut tourner de 90° vers la gauche

    // Normaliser l'angle final entre -180 et 180
    if (angleFinal < -180.0)
        angleFinal += 360.0;
    if (angleFinal > 180.0)
        angleFinal -= 360.0;

    LOG_SYSTEM("Début rotation: Initial=%.1f°, Objectif=%.1f°", angleInitial, angleFinal);

    const int VITESSE_ROTATION = 120;  // Vitesse modérée pour plus de précision
    const float ANGLE_TOLERANCE = 3.0; // Tolérance légèrement augmentée
    TickType_t debut = xTaskGetTickCount();

    // Démarrer la rotation
    tournerGauche();
    analogWrite(ENA1, VITESSE_ROTATION);
    analogWrite(ENB1, VITESSE_ROTATION);

    float angleActuel = yaw; // Capture de la valeur actuelle
    LOG_SYSTEM("Angle actuel: %.1f°", angleActuel);

    // Petit délai pour ne pas surcharger le processeur
    vTaskDelay(pdMS_TO_TICKS(10));

    return false;
}

// Variables pour la supervision
volatile bool isCalibrating = false;

// Tâche de supervision
void TaskSupervision(void *pvParameters)
{
    pinMode(LED_SUPERVISION, OUTPUT);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t blinkInterval = BLINK_CALIBRATION;
    bool ledState = false;
    
    while(true) {
        if(isCalibrating) {
            // Mode calibration : clignotement rapide
            blinkInterval = BLINK_CALIBRATION;
        } else {
            // Choix du pattern selon l'état
            RobotState state = getRobotState();
            switch(state) {
                case RUNNING:
                    blinkInterval = BLINK_RUNNING;
                    break;
                case PICKING:
                    blinkInterval = BLINK_PICKING;
                    break;
                case ROTATION_GAUCHE:
                    blinkInterval = BLINK_ROTATION;
                    break;
                default:
                    blinkInterval = BLINK_RUNNING;
            }
        }
        
        // Inversion de l'état de la LED
        ledState = !ledState;
        digitalWrite(LED_SUPERVISION, ledState);
        
        // Délai jusqu'au prochain changement d'état
        vTaskDelay(pdMS_TO_TICKS(blinkInterval));
    }
}

// Fonction pour normaliser l'angle entre -180 et 180 degrés
float normalizeAngle180(float raw_data, float cap_reference) {
    float normalized = raw_data - cap_reference;
    normalized = fmod(normalized + 180.0, 360.0) - 180.0;
    return normalized;
}

// Fonction pour normaliser l'angle entre 0 et 360 degrés
float normalizeAngle360(float raw_data, float cap_reference) {
    float normalized = raw_data - cap_reference;
    normalized = fmod(normalized, 360.0);
    if (normalized < 0) normalized += 360.0;
    return normalized;
}

// Tâche de contrôle du gyroscope
void TaskGyroControl(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 10ms
    LOG_GYRO("[Gyro] Démarrage tâche gyroscope");

    while (true)
    {
        // Lecture et traitement du MPU6050
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        gyroZ = g.gyro.z - gyroZ_offset;

        // Zone morte pour réduire le bruit
        if (abs(gyroZ) < 0.01)
        {
            gyroZ = 0;
        }

        double elapsedTime = (double)xFrequency / 1000.0;
        double yaw_rate = gyroZ;

        // Calculs du filtre Kalman
        static double drift_accumulator = 0;
        drift_accumulator += yaw_rate * elapsedTime;

        static double yaw_measurement = 0.0;
        yaw_measurement += (yaw_rate - (drift_accumulator * drift_correction)) * elapsedTime;

        dk.predict(elapsedTime, yaw_rate);
        dk.update(yaw_measurement, Measurement_Noise_Covariance);

        // Conversion des données brutes en degrés
        yaw = dk.pos() * (180.0 / PI);
        
        // Gestion des états du robot
        RobotState currentState = getRobotState();
        double correction = 0.0; // Déclaré en dehors du switch
        
        // Normalisation de l'angle selon l'état
        float normalized_yaw;

        switch (currentState)
        {
        case RUNNING:
            // Utilisation de la normalisation -180 à 180 pour le contrôle en ligne droite
            normalized_yaw = normalizeAngle180(yaw, cap_reference);
            correction = KP * normalized_yaw; // L'erreur est déjà relative au cap de référence
            update_motor_speed(correction);
            avancer();
            break;

        case OBSTACLE:
            // Début de la rotation
            LOG_SYSTEM("Obstacle détecté: début rotation");
            yaw_initial = yaw;
            rotationEnCours = true;
            setRobotState(ROTATION_GAUCHE);
            break;

        case ROTATION_GAUCHE:
            if (rotationEnCours)
            {
                // Appliquer la rotation
                tournerDroite();
                analogWrite(ENA1, 80); // Vitesse de rotation modérée
                analogWrite(ENB1, 80);

                // Utiliser la normalisation appropriée selon la rotation
                float deltaYaw;
                if (cap_reference >= 180.0) {
                    // Pour le troisième virage et plus, utiliser 0-360
                    deltaYaw = normalizeAngle360(yaw, yaw_initial);
                } else {
                    // Pour les deux premiers virages, utiliser -180 à 180
                    deltaYaw = normalizeAngle180(yaw, yaw_initial);
                }

                float erreur = 90 - deltaYaw; // Erreur = angle cible - angle actuel

                LOG_SYSTEM("Rotation en cours: %.1f degrés, erreur: %.1f", deltaYaw, erreur);

                // Si l'erreur est suffisamment petite (tolérance de 2 degrés)
                if (abs(erreur) <= 2.0)
                {
                    // Mettre à jour le cap de référence
                    cap_reference += 90.0;
                    // Normaliser cap_reference entre -180 et +180
                    cap_reference = fmod(cap_reference + 180.0, 360.0) - 180.0;
                    LOG_SYSTEM("Nouveau cap de référence: %.1f°", cap_reference);

                    // Fin de la rotation
                    LOG_SYSTEM("Rotation terminée avec erreur: %.1f°", erreur);
                    arreter();
                    rotationEnCours = false;
                    nombreVirages++;
                    LOG_SYSTEM("Nombre de virages: %d", nombreVirages);
                    
                    if (nombreVirages >= 2) {
                        LOG_SYSTEM("2 virages effectués, activation mode manœuvre spéciale");
                    }
                    
                    setRobotState(RUNNING);
                    xTaskNotifyGive(ultrasonicTaskHandle);
                }
            }
            break;

        default:
            // Autres états (STOPPING, PICKING)
            break;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Tâche de détection de jetons
void TaskDetectJeton(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    LOG_DETECTION("[Détection] Démarrage tâche détection");

    while (true)
    {
        if (getRobotState() == RUNNING)
        {
            int irGauche = digitalRead(IR_GAUCHE);
            int irDroite = digitalRead(IR_DROITE);

            if (irGauche == HIGH || irDroite == HIGH)
            {
                LOG_DETECTION("Jeton détecté! Capteurs: Gauche=%d, Droite=%d",
                              irGauche, irDroite);

                setRobotState(STOPPING);
                LOG_DETECTION("Changement état -> STOPPING");

                arreter();
                LOG_DETECTION("Moteurs arrêtés");

                LOG_DETECTION("Notification vers tâche ramassage");
                xTaskNotifyGive(pickupTaskHandle);

                LOG_DETECTION("Attente fin ramassage...");
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                LOG_DETECTION("Reprise reçue");

                update_motor_speed(0);
                setRobotState(RUNNING);
                LOG_DETECTION("Reprise marche normale");
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Tâche de ramassage de jetons
void TaskRamassageJeton(void *pvParameters)
{
    LOG_PICKUP("[Ramassage] Démarrage tâche ramassage");

    while (true)
    {
        LOG_PICKUP("[Ramassage] Attente d'une détection...");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        LOG_PICKUP("[Ramassage] Notification reçue de la tâche détection");

        setRobotState(PICKING);
        LOG_PICKUP("[Ramassage] État -> PICKING");

        LOG_PICKUP("[Ramassage] Début séquence servo -> position ramassage");
        myservo.write(SERVO_PICKUP_POS);
        vTaskDelay(pdMS_TO_TICKS(1000));

        LOG_PICKUP("[Ramassage] Servo -> position initiale");
        myservo.write(SERVO_INIT_POS);
        vTaskDelay(pdMS_TO_TICKS(500));

        update_motor_speed(0);
        setRobotState(RUNNING);
        LOG_PICKUP("[Ramassage] Reset vitesses et état -> RUNNING");

        LOG_PICKUP("[Ramassage] Notification de fin vers tâche détection");
        xTaskNotifyGive(detectionTaskHandle);
        LOG_PICKUP("[Ramassage] Cycle de ramassage terminé");
    }
}

// Tâche de détection ultrasonique
void TaskUltrasonicDetection(void *pvParameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 50ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    LOG_SYSTEM("[Ultrasonic] Démarrage tâche détection ultrasonique");

    while (true)
    {
        if (getRobotState() == RUNNING)
        {
            // Utilisation de ping_median avec 5 échantillons
            unsigned int distance = sonar.ping_median(5);
            distance = sonar.convert_cm(distance);

            // Log périodique
            static uint32_t lastLog = 0;
            if (millis() - lastLog > 1000)
            { // Log toutes les secondes
                LOG_SYSTEM("[Ultrasonic] Distance: %d cm", distance);
                lastLog = millis();
            }

            // Vérification de la distance
            if (distance > 0 && distance <= 20)
            {
                LOG_SYSTEM("[Ultrasonic] Obstacle détecté à %d cm", distance);
                
                if (nombreVirages >= 2 && !maneuverInProgress) {
                    LOG_SYSTEM("[Ultrasonic] Activation manœuvre d'évitement");
                    setRobotState(OBSTACLE);
                    xTaskNotifyGive(maneuverTaskHandle);
                } else {
                    setRobotState(OBSTACLE);
                    // Attendre la notification de fin de rotation
                    LOG_SYSTEM("[Ultrasonic] Attente fin de rotation...");
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                LOG_SYSTEM("[Ultrasonic] Rotation terminée, reprise scan");
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
}

// Tâche de manœuvre d'évitement
void TaskManeuver(void *pvParameters)
{
    while (true)
    {
        // Attendre une notification pour démarrer la manœuvre
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        maneuverInProgress = true;
        
        // Suspendre les autres tâches sauf la supervision
        vTaskSuspend(gyroTaskHandle);
        vTaskSuspend(detectionTaskHandle);
        vTaskSuspend(pickupTaskHandle);
        vTaskSuspend(ultrasonicTaskHandle);
        
        LOG_SYSTEM("Début manœuvre d'évitement");
        
        // 1. Tourner à droite pendant 2 secondes
        LOG_SYSTEM("Rotation droite");
        tournerGauche();
        analogWrite(ENA1, MANEUVER_SPEED);
        analogWrite(ENB1, MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(MANEUVER_TURN_TIME));
        
        // 2. Avancer pendant 1 seconde
        LOG_SYSTEM("Avance directe");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENA1, MANEUVER_SPEED);
        analogWrite(ENB1, MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(MANEUVER_FORWARD_TIME));
        
        // 3. Reculer pendant 2 secondes
        LOG_SYSTEM("Marche arrière");
        reculer();
        analogWrite(ENA1, MANEUVER_SPEED);
        analogWrite(ENB1, MANEUVER_SPEED);
        vTaskDelay(pdMS_TO_TICKS(MANEUVER_BACK_TIME));
        
        // Arrêter les moteurs
        arreter();
        LOG_SYSTEM("Fin de la manœuvre d'évitement");
        
        // Arrêt complet du robot après la manœuvre
        nombreVirages = 0;
        maneuverInProgress = false;
        setRobotState(STOPPING);
        
        // Le robot reste dans cet état jusqu'à un redémarrage manuel
        LOG_SYSTEM("Robot arrêté après manœuvre d'évitement - Redémarrage manuel nécessaire");
        while(1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void setup()
{
    Serial.begin(115200);
    LOG_SYSTEM("===== Démarrage du robot =====");

    // Initialisation du mutex
    motorMutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    if (motorMutex == NULL || stateMutex == NULL)
    {
        LOG_ERROR("Échec création mutex!");
        while (1)
            ;
    }
    LOG_SYSTEM("Mutex initialisés");

    // Configuration du MPU6050
    if (!mpu.begin())
    {
        LOG_ERROR("MPU6050 non détecté!");
        while (1)
            ;
    }
    LOG_SYSTEM("MPU6050 détecté");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    LOG_SYSTEM("MPU6050 configuré");

    // Création de la tâche de supervision sur le Core 0
    xTaskCreatePinnedToCore(
        TaskSupervision,
        "Supervision",
        2048,
        NULL,
        1,
        &supervisionTaskHandle,
        0  // Core 0
    );
    LOG_SYSTEM("Tâche Supervision créée sur Core 0");

    // Configuration des pins
    pinMode(IR_GAUCHE, INPUT);
    pinMode(IR_DROITE, INPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA1, OUTPUT);
    pinMode(ENB1, OUTPUT);
    LOG_SYSTEM("Pins configurées");

    // Configuration du servo
    myservo.attach(SERVO_PIN);
    myservo.write(SERVO_INIT_POS);
    LOG_SYSTEM("Servo initialisé");

    // Calibration du gyroscope
    LOG_SYSTEM("Début calibration gyroscope");
    isCalibrating = true;  // Début de la calibration
    delay(200);

    LOG_SYSTEM("Collection de %d échantillons...", CALIBRATION_SAMPLES);
    for (int number_of_samples = 0; number_of_samples < CALIBRATION_SAMPLES; number_of_samples++)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        gyroX_cal += g.gyro.x;
        gyroY_cal += g.gyro.y;
        gyroZ_cal += g.gyro.z;

        if (number_of_samples % 500 == 0)
        {
            LOG_SYSTEM("Progrès: %d%%", (number_of_samples * 100) / CALIBRATION_SAMPLES);
        }

        delay(2);
    }

    gyroX_offset = gyroX_cal / CALIBRATION_SAMPLES;
    gyroY_offset = gyroY_cal / CALIBRATION_SAMPLES;
    gyroZ_offset = gyroZ_cal / CALIBRATION_SAMPLES;

    LOG_SYSTEM("Offsets: X=%.6f, Y=%.6f, Z=%.6f",
               gyroX_offset, gyroY_offset, gyroZ_offset);
    LOG_SYSTEM("Calibration terminée!");
    isCalibrating = false;  // Fin de la calibration

    // Création des tâches
    LOG_SYSTEM("Création des tâches FreeRTOS");

    // Création de la tâche de manœuvre d'évitement
    xTaskCreatePinnedToCore(
        TaskManeuver,
        "Maneuver",
        2048,
        NULL,
        3, // Priorité égale à GyroControl
        &maneuverTaskHandle,
        1  // Core 1
    );
    LOG_SYSTEM("Tâche Manœuvre créée");

    xTaskCreatePinnedToCore(
        TaskSupervision,
        "Supervision",
        2048,
        NULL,
        1, // Priorité basse
        &supervisionTaskHandle,
        1 // Core 1
    );
    LOG_SYSTEM("Tâche Supervision créée (priorité 1)");

    xTaskCreatePinnedToCore(
        TaskGyroControl,
        "GyroControl",
        4096,
        NULL,
        3, // Priorité 2
        &gyroTaskHandle,
        1 // Core 1
    );
    LOG_SYSTEM("Tâche Gyro créée (priorité 3)");

    xTaskCreatePinnedToCore(
        TaskDetectJeton,
        "DetectJeton",
        2048,
        NULL,
        2, // Priorité 1
        &detectionTaskHandle,
        1 // Core 1
    );
    LOG_SYSTEM("Tâche Détection créée (priorité 2)");

    xTaskCreatePinnedToCore(
        TaskRamassageJeton,
        "RamassageJeton",
        2048,
        NULL,
        4, // Priorité 4
        &pickupTaskHandle,
        1 // Core 1
    );
    LOG_SYSTEM("Tâche Ramassage créée (priorité 4)");

    // Création de la tâche de détection ultrasonique
    xTaskCreatePinnedToCore(
        TaskUltrasonicDetection,
        "UltrasonicDetect",
        2048,
        NULL,
        1,                     // Priorité basse
        &ultrasonicTaskHandle, // Sauvegarder le handle
        1                      // Core 1
    );
    LOG_SYSTEM("Tâche Détection ultrasonique créée");

    // Création de la tâche de manœuvre d'évitement
    xTaskCreatePinnedToCore(
        TaskManeuver,
        "Maneuver",
        2048,
        NULL,
        1,                     // Priorité basse
        &maneuverTaskHandle, // Sauvegarder le handle
        1                      // Core 1
    );
    LOG_SYSTEM("Tâche Manœuvre d'évitement créée");

    LOG_SYSTEM("===== Initialisation terminée =====\n");
}

void loop()
{
    // Le loop() reste vide car tout est géré par les tâches FreeRTOS
    vTaskDelay(pdMS_TO_TICKS(1000));
} ////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////