#include <mbed.h>
#include <threadLvgl.h>
#include "demos/lv_demos.h"
#include <cstdio>

ThreadLvgl threadLvgl(30);

// Configuration des pins pour les servomoteurs
PwmOut servoPin1(PA_8);
PwmOut servoPin2(PB_15);

// Constantes pour la période du servo et les positions
const int SERVO_PERIOD_MS = 20; // Période du servo en ms

// Positions initiales des servomoteurs (à ajuster selon vos servomoteurs)
const float INITIAL_POSITION_1 = 0.5f;
const float INITIAL_POSITION_2 = 0.5f;

// Positions des servomoteurs pour le quart de tour (à ajuster selon vos servomoteurs)
const float QUARTER_TURN_POSITION_1 = 1.0f;
const float QUARTER_TURN_POSITION_2 = 0.0f;

// Variables pour stocker les valeurs de mesure en degrés
float servoAngle1 = 0.0f;
float servoAngle2 = 0.0f;

// Fonction de rappel pour le premier bouton LVGL
static void btn_event_cb1(lv_event_t * e) {
    servoPin1.pulsewidth_ms(1.5); // Pulse width pour 90 degrés (ajustez cette valeur selon votre servomoteur)
    servoAngle1 = 90.0f; // Met à jour la valeur de mesure en degrés
}

// Fonction de rappel pour le deuxième bouton LVGL
static void btn_event_cb2(lv_event_t * e) {
    servoPin2.pulsewidth_ms(1.5); // Pulse width pour 90 degrés (ajustez cette valeur selon votre servomoteur)
    servoAngle2 = 90.0f; // Met à jour la valeur de mesure en degrés
}

int main() {
    // Initialiser les signaux PWM pour les servomoteurs
    servoPin1.period_ms(SERVO_PERIOD_MS);
    servoPin1.pulsewidth_ms(INITIAL_POSITION_1); // Position initiale du servomoteur 1
    servoPin2.period_ms(SERVO_PERIOD_MS);
    servoPin2.pulsewidth_ms(INITIAL_POSITION_2); // Position initiale du servomoteur 2

    // Initialiser LVGL
    threadLvgl.lock();

    // Créer une démo de widgets LVGL
    lv_demo_widgets();

    // Créer un premier bouton sur l'écran pour le premier servomoteur
    lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(btn1, 50, 50);
    lv_obj_set_size(btn1, 120, 50);
    lv_obj_add_event_cb(btn1, btn_event_cb1, LV_EVENT_CLICKED, NULL);
    lv_obj_t * label1 = lv_label_create(btn1);
    lv_label_set_text(label1, "Servo 1");
    lv_obj_center(label1);

    // Créer un label pour afficher la valeur de mesure du premier servomoteur
    lv_obj_t * servo1_label = lv_label_create(lv_scr_act());
    lv_obj_align(servo1_label, btn1, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_label_set_text_fmt(servo1_label, "Angle: %.1f deg", servoAngle1);

    // Créer un deuxième bouton sur l'écran pour le deuxième servomoteur
    lv_obj_t * btn2 = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(btn2, 50, 150);
    lv_obj_set_size(btn2, 120, 50);
    lv_obj_add_event_cb(btn2, btn_event_cb2, LV_EVENT_CLICKED, NULL);
    lv_obj_t * label2 = lv_label_create(btn2);
    lv_label_set_text(label2, "Servo 2");
    lv_obj_center(label2);

    // Créer un label pour afficher la valeur de mesure du deuxième servomoteur
    lv_obj_t * servo2_label = lv_label_create(lv_scr_act());
    lv_obj_align(servo2_label, btn2, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_label_set_text_fmt(servo2_label, "Angle: %.1f deg", servoAngle2);

    threadLvgl.unlock();

    while (1) {
        // Boucle principale
        ThisThread::sleep_for(10ms);
    }
}
