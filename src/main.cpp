#include <mbed.h> // Inclut la bibliothèque mbed
#include <threadLvgl.h> // Inclut la bibliothèque threadLvgl pour l'interface graphique
#include "demos/lv_demos.h" // Inclut les démos LVGL pour l'interface graphique
#include <cstdio> // Inclut la bibliothèque standard de gestion des E/S

ThreadLvgl threadLvgl(30); // Initialise un thread LVGL avec une période de 30 ms

// Configuration des pins pour les servomoteurs et les capteurs
PwmOut servoPin1(PA_8); // Initialise la sortie PWM pour le servomoteur 1 sur la broche PA_8 (D10)
PwmOut servoPin2(PB_15); // Initialise la sortie PWM pour le servomoteur 2 sur la broche PB_15 (D11)
DigitalIn sensor1(D2); // Initialise l'entrée numérique pour le capteur 1 sur la broche D2
DigitalIn sensor2(D3); // Initialise l'entrée numérique pour le capteur 2 sur la broche D3
DigitalIn sensor3(D4); // Initialise l'entrée numérique pour le capteur 3 sur la broche D4
DigitalIn sensor4(D5); // Initialise l'entrée numérique pour le capteur 4 sur la broche D5

// Constantes pour la période du servo et les positions
const int SERVO_PERIOD_MS = 20; // Période du servo en millisecondes

// Constantes pour le servomoteur FT90M
const float FT90M_pulseMin = 0.001; // Impulsion pour pivoter le servo FT90M à 0º (1000 μs)
const float FT90M_pulseMax = 0.002; // Impulsion pour pivoter le servo FT90M à 280º (2000 μs)

// Constantes pour les servomoteurs standards
const float pulseMin = 0.00065; // Impulsion pour pivoter le servo standard à 0º (650 μs)
const float pulseMax = 0.001275; // Impulsion pour pivoter le servo standard à 90º (1275 μs)
// const float pulseMax = 0.00255; // Impulsion pour pivoter le servo standard à 180º (2550 μs) - commentaire

// Variables pour stocker les valeurs de mesure en degrés
int angle1 = 0; // Initialisé à la position correspondante pour le servomoteur 1
int angle2 = 0; // Initialisé à la position correspondante pour le servomoteur 2

// Déclaration des objets LVGL pour les labels
lv_obj_t *servo1_label; // Label pour afficher l'angle du servomoteur 1
lv_obj_t *servo2_label; // Label pour afficher l'angle du servomoteur 2
lv_obj_t *sensor1_label; // Label pour afficher l'état du capteur 1
lv_obj_t *sensor2_label; // Label pour afficher l'état du capteur 2
lv_obj_t *sensor3_label; // Label pour afficher l'état du capteur 3
lv_obj_t *sensor4_label; // Label pour afficher l'état du capteur 4
lv_obj_t *car_detected_label; // Label pour afficher le message de détection de voiture pour le capteur 1
lv_obj_t *car_detected_label2; // Label pour afficher le message de détection de voiture pour le capteur 2

// Fonction pour mapper les valeurs (similaire à la fonction Arduino map)
float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Fonction pour définir l'angle du servo standard
void setServoAngle(PwmOut &servoPin, int angle) {
    float pulseWidth = map(angle, 0, 180, pulseMin, pulseMax);
    servoPin.pulsewidth(pulseWidth);
}

// Fonction pour définir l'angle du servomoteur FT90M
void setServoAngleFT90M(PwmOut &servoPin, int angle) {
    float pulseWidth = map(angle, 0, 280, FT90M_pulseMin, FT90M_pulseMax);
    servoPin.pulsewidth(pulseWidth);
}

// Fonction pour mettre à jour le label de position du servomoteur
void updateServoLabel(lv_obj_t *label, int angle) {
    lv_label_set_text_fmt(label, "Angle: %d deg", angle);
}

// Fonction pour mettre à jour le label du capteur
void updateSensorLabel(lv_obj_t *label, const char* text) {
    lv_label_set_text(label, text);
}

// Fonction pour faire bouger le servo de 0 à 180 degrés 
void moveServo_ouvert(PwmOut &servoPin, lv_obj_t *label) {
    for (int pos = 0; pos <= 180; pos += 1) { // Boucle pour faire tourner le servo de 0 à 180 degrés
        setServoAngle(servoPin, pos); // Définir l'angle du servo
        ThisThread::sleep_for(10ms); // Attendre 10 ms
        updateServoLabel(label, pos); // Mettre à jour le label avec l'angle actuel
    }
}    
// Fonction pour faire bouger le servo de 180 à 0 degrés
void moveServo_fermer(PwmOut &servoPin, lv_obj_t *label){
    for (int pos = 180; pos >= 0; pos -= 1) { // Boucle pour faire tourner le servo de 180 à 0 degrés
        setServoAngle(servoPin, pos); // Définir l'angle du servo
        ThisThread::sleep_for(10ms); // Attendre 10 ms
        updateServoLabel(label, pos); // Mettre à jour le label avec l'angle actuel
    }
}

// Fonction pour bouger le servo à 90 degrés et revenir
void moveServoTo90AndBack(PwmOut &servoPin, lv_obj_t *label) {
    setServoAngle(servoPin, 90); // Définir l'angle du servo à 90 degrés
    updateServoLabel(label, 90); // Mettre à jour le label avec l'angle actuel
    ThisThread::sleep_for(20ms); // Attendre 20 ms
    setServoAngle(servoPin, 0); // Revenir à 0 degré
    updateServoLabel(label, 0); // Mettre à jour le label avec l'angle actuel
}

// Fonction de test pour le premier bouton LVGL
static void btn_event_cb1(lv_event_t *e) {
    moveServo_ouvert(servoPin1, servo1_label); // Faire bouger le servomoteur 1 de 0 à 180 degrés 
    ThisThread::sleep_for(5000ms); // Attendre 5s
    moveServo_fermer(servoPin1, servo1_label);// Faire bouger le servomoteur 1 de 180 à 0 degrés 
}

// Fonction de test pour le deuxième bouton LVGL
static void btn_event_cb2(lv_event_t *e) {
    moveServo_ouvert(servoPin2, servo2_label); // Faire bouger le servomoteur 2 de 0 à 180 degrés 
    ThisThread::sleep_for(5000ms); // Attendre 5s 
    moveServo_fermer(servoPin2, servo2_label);// Faire bouger le servomoteur 2 de 180 à 0 degrés 
}

// Fonction pour créer le premier bouton sur l'écran pour le premier servomoteur
void create_button1() {
    lv_obj_t *btn1 = lv_btn_create(lv_scr_act()); // Créer un bouton btn1
    lv_obj_set_pos(btn1, 50, 50); // Positionner le bouton à la position X=50 et Y=50
    lv_obj_set_size(btn1, 120, 50); // Définir la taille du bouton de longueur=120 et largeur=50
    lv_obj_add_event_cb(btn1, btn_event_cb1, LV_EVENT_CLICKED, NULL); // Ajouter un appel d'événement pour le bouton 1
    lv_obj_t *label1 = lv_label_create(btn1); // Créer un label pour le bouton
    lv_label_set_text(label1, "Servo 1"); // Définir le texte du label
    lv_obj_center(label1); // Centrer le label dans le bouton
}

// Fonction pour créer le deuxième bouton sur l'écran pour le deuxième servomoteur
void create_button2() {
    lv_obj_t *btn2 = lv_btn_create(lv_scr_act()); // Créer un bouton
    lv_obj_set_pos(btn2, 50, 150); // Positionner le bouton à la position X=50 et Y=150
    lv_obj_set_size(btn2, 120, 50); // Définir la taille du bouton longueur=120 et largeur=50
    lv_obj_add_event_cb(btn2, btn_event_cb2, LV_EVENT_CLICKED, NULL); // Ajouter un rappel d'événement pour le bouton
    lv_obj_t *label2 = lv_label_create(btn2); // Créer un label pour le bouton
    lv_label_set_text(label2, "Servo 2"); // Définir le texte du label
    lv_obj_center(label2); // Centrer le label dans le bouton
}

int main() {
    // Initialiser les signaux PWM pour les servomoteurs
    servoPin1.period_ms(SERVO_PERIOD_MS); // Définir la période PWM pour le servomoteur 1
    setServoAngleFT90M(servoPin1, 0); // Position initiale du servomoteur FT90M à 0 degré

    servoPin2.period_ms(SERVO_PERIOD_MS); // Définir la période PWM pour le servomoteur 2
    setServoAngleFT90M(servoPin2, 0); // Position initiale du servomoteur FT90M à 0 degré

    // Attendre un moment pour assurer la stabilité des servos
    ThisThread::sleep_for(500ms); // Attendre 500 ms

    // Initialiser la bibliothèque LVGL
    lv_init();
    threadLvgl.lock(); // Verrouiller le thread LVGL pour initialiser les objets

    /** Créer les boutons pour contrôler les servomoteurs**/
    create_button1(); // Créer le bouton pour le premier servomoteur
    create_button2(); // Créer le bouton pour le deuxième servomoteur

    /**Créer les labels pour afficher les angles des servomoteurs et les états des capteurs**/
    
    servo1_label = lv_label_create(lv_scr_act()); // Créer un label pour le servomoteur 1
    lv_obj_align(servo1_label, LV_ALIGN_TOP_LEFT, 10, 10); // Aligner le label
    updateServoLabel(servo1_label, angle1); // Mettre à jour le label avec l'angle initial du servomoteur 1

    servo2_label = lv_label_create(lv_scr_act()); // Créer un label pour le servomoteur 2
    lv_obj_align(servo2_label, LV_ALIGN_TOP_LEFT, 10, 30); // Aligner le label
    updateServoLabel(servo2_label, angle2); // Mettre à jour le label avec l'angle initial du servomoteur 2


    /**Créer et positionner 2 labels distincts pour les capteur1 et capteur2 **/

    //Créer et positionner d'un label pour le capteur1
    sensor1_label = lv_label_create(lv_scr_act());
    lv_obj_align(sensor1_label, LV_ALIGN_TOP_LEFT, 10, 50);
    updateSensorLabel(sensor1_label, "Sensor 1: 0");

    //Créer et positionner d'un label pour le capteur2
    sensor2_label = lv_label_create(lv_scr_act());
    lv_obj_align(sensor2_label, LV_ALIGN_TOP_LEFT, 10, 70);
    updateSensorLabel(sensor2_label, "Sensor 2: 0");

    //Créer et positionner d'un label pour le capteur3
    sensor3_label = lv_label_create(lv_scr_act());
    lv_obj_align(sensor3_label, LV_ALIGN_TOP_LEFT, 10, 90);
    updateSensorLabel(sensor3_label, "Sensor 3: 0");

    //Créer et positionner d'un label pour le capteur4
    sensor4_label = lv_label_create(lv_scr_act());
    lv_obj_align(sensor4_label, LV_ALIGN_TOP_LEFT, 10, 110);
    updateSensorLabel(sensor4_label, "Sensor 4: 0");

    // Créer les labels pour la détection des voitures
    car_detected_label = lv_label_create(lv_scr_act());
    lv_obj_align(car_detected_label, LV_ALIGN_TOP_LEFT, 10, 130);
    updateSensorLabel(car_detected_label, "Car detected: No");

    car_detected_label2 = lv_label_create(lv_scr_act());
    lv_obj_align(car_detected_label2, LV_ALIGN_TOP_LEFT, 10, 150);
    updateSensorLabel(car_detected_label2, "Car detected 2: No");

    threadLvgl.unlock(); // Déverrouiller le thread LVGL pour permettre les mises à jour

while (1) {

    /**Détection des voitures par les capteurs 1 et 2 **/ 
    
    if (sensor1.read() == 0) {// Détection de voiture par le capteur 1
        // Si le capteur 1 détecte une voiture (signal bas)
        updateSensorLabel(sensor1_label, "Sensor 1: 0");
        updateSensorLabel(car_detected_label, "Voiture detectee");
    } else {
        // Si le capteur 1 ne détecte pas de voiture (signal haut)
        updateSensorLabel(sensor1_label, "Sensor 1: 1");
        updateSensorLabel(car_detected_label, "Voiture non detectee");
    }

    // Détection de voiture par le capteur 2
    if (sensor2.read() == 0) {
        // Si le capteur 2 détecte une voiture (signal bas)
        updateSensorLabel(sensor2_label, "Sensor 2: 0");
        updateSensorLabel(car_detected_label2, "Voiture detectee");
    } else {
        // Si le capteur 2 ne détecte pas de voiture (signal haut)
        updateSensorLabel(sensor2_label, "Sensor 2: 1");
        updateSensorLabel(car_detected_label2, "Voiture non detectee");
    }

    /**Mouvement du servo 1 et servo 2 lors de la détection des capteurs 4 et 3**/ 

    // Mouvement d'ouverture de barrière du servo 1 si détection sur le capteur 4
    if (sensor4.read() == 0) {// Si le capteur 4 est activé (signal à 0)
        moveServo_ouvert(servoPin1, servo1_label);// Faire tourner le servomoteur 1 de 0 à 180 degrés
        setServoAngle(servoPin1, 0);// Revenir à la position d'origine
        updateServoLabel(servo1_label, 0); // Mettre à jour le label du servomoteur 1
    }

// Mouvement de fermeture de barrière du servo 1 si absence de détection sur les capteur 4 et 1
    else if ((sensor4.read() && sensor1.read()  == 1) {// Si le capteur 4  et 1 sont désactivés (signal à 1)
        moveServo_fermer(servoPin1, servo1_label);// Faire tourner le servomoteur 1 de 180 à 0 degrés
        setServoAngle(servoPin1, 0);// Revenir à la position d'origine
        updateServoLabel(servo1_label, 0); // Mettre à jour le label du servomoteur 1
    }
    
    // Mouvement d'ouverture de barrière du servo 2 si détection sur le capteur 3
    if (sensor3.read()  == 0) {// Si le capteur 3 est activé (signal à 0)
        moveServo_ouvert(servoPin2, servo2_label);// Faire tourner le servomoteur 2 de 0 à 180 degrés
        setServoAngle(servoPin2, 0);// Revenir à la position d'origine
        updateServoLabel(servo2_label, 0); // Mettre à jour le label du servomoteur 2
    }


 // Mouvement de fermeture de barrière du servo 2 si absence de détection sur les capteur 3 et 2
    else if ((sensor3.read() && sensor2.read()  == 1) {// Si le capteur 3 et 2 sont désactivés (signal à 1)
        moveServo_fermer(servoPin2, servo2_label);// Faire tourner le servomoteur 2 de 180 à 0 degrés
        setServoAngle(servoPin2, 0);// Revenir à la position d'origine
        updateServoLabel(servo2_label, 0); // Mettre à jour le label du servomoteur 2
    }

            
    /** Mettre à jour les labels des 4 capteurs**/
    
    if (sensor1.read() == 0) {
        // Mettre à jour le label du capteur 1 si le signal est à 0
        updateSensorLabel(sensor1_label, "Sensor 1: 0");
    } else {
        // Mettre à jour le label du capteur 1 si le signal est à 1
        updateSensorLabel(sensor1_label, "Sensor 1: 1");
    }
    
    if (sensor2.read() == 0) {
        // Mettre à jour le label du capteur 2 si le signal est à 0
        updateSensorLabel(sensor2_label, "Sensor 2: 0");
    } else {
        // Mettre à jour le label du capteur 2 si le signal est à 1
        updateSensorLabel(sensor2_label, "Sensor 2: 1");
    }

    if (sensor3.read() == 0) {
        // Mettre à jour le label du capteur 3 si le signal est à 0
        updateSensorLabel(sensor3_label, "Sensor 3: 0");
    } else {
        // Mettre à jour le label du capteur 3 si le signal est à 1
        updateSensorLabel(sensor3_label, "Sensor 3: 1");
    }

    if (sensor4.read() == 0) {
        // Mettre à jour le label du capteur 4 si le signal est à 0
        updateSensorLabel(sensor4_label, "Sensor 4: 0");
    } else {
        // Mettre à jour le label du capteur 4 si le signal est à 1
        updateSensorLabel(sensor4_label, "Sensor 4: 1");
    }

    // Boucle principale
    ThisThread::sleep_for(20ms); // Pause de 20 millisecondes 

  }
}
