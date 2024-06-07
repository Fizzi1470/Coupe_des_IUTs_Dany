// ======================= Parametres =======================//

#define SPEED 0.2
#define KP 0.35
#define KD 0.25

#define SEUIL_PRIORITE 20 //cm noooormalement

#define ATTENTE_APRES_VIRAGE 200

#define SEUIL_LIGNE_PERDUE 300
#define SEUIL_DETECTION_VIRAGE 500

// ======================= Dépendances =======================//

#include "m3pi.h"
#include "mbed.h"

// ======================= Définitions =======================//

Serial pc(USBTX, USBRX, 115200);

m3pi pi;
char message[50]; // affichage ecran
 
DigitalIn BTN(p21,PullUp);

uint16_t sensors[5];
float line_pos = 0;

bool line_left = false, line_right = false, crossing = false, line_detected = false;
bool lost_line = false;

AnalogIn sensor_port(p17);

// ======================= Utilitaires =======================//

#define ON_LINE (sensors[0] > SEUIL_LIGNE_PERDUE || sensors[1] > SEUIL_LIGNE_PERDUE || sensors[2] > SEUIL_LIGNE_PERDUE || sensors[3] > SEUIL_LIGNE_PERDUE || sensors[4] > SEUIL_LIGNE_PERDUE)
#define LINE_RIGHT (sensors[2] > SEUIL_DETECTION_VIRAGE && sensors[4] > SEUIL_DETECTION_VIRAGE)
#define LINE_LEFT (sensors[0] > SEUIL_DETECTION_VIRAGE && sensors[2] > SEUIL_DETECTION_VIRAGE)  

float distance(){
    float t = sensor_port.read() * 3.3;
    return (13.07143 * t * t * t * t - 77.17951 * t * t * t + 162.49768 * t * t - 150.403 * t + 64.30365);
}

float constrain(float data, float floor, float ceilling){
    if (data > ceilling) return ceilling;
    else if (data < floor) return floor;
    else return data;
}

void perpandicular_turn(bool dir = 1){ // 1 pour tourner vers la droite, 0 pour tourner vers la gauche // fait faire un quart de tour 
    pi.stop();
    int8_t coef = (dir ? 1 : -1);
    pi.left_motor(coef* 0.3);
    pi.right_motor(-coef*0.3);
    wait_ms(210);
    pi.stop();
    wait_ms(ATTENTE_APRES_VIRAGE);
}
void u_turn(bool dir = 1){ // fait faire demi tour 
    perpandicular_turn(dir);
    perpandicular_turn(dir);

    wait_ms(200);
}

void wait_button_press(){while(BTN);while(!BTN);} // attend un appui sur le bouton de manière bloquante (attention à son utilisation)

// ======================= Fonctions =======================//

int compteur_ligne_droite = 0;
int compteur_ligne_gauche = 0;
int compteur_croisements = 0;
int compteur_fin_de_ligne = 0;

float old_error = 0;

void setup(){ // appelé une fois au démarrage du programme
    wait_button_press();
}

void loop(){ // appelé en boucle
// suiveur de ligne 
    float error = line_pos;

    float delta = error - old_error;

    float consigne = error*KP + delta*KD;

    old_error = error;

    if((!ON_LINE) || LINE_LEFT || LINE_RIGHT) consigne = 0;

    pi.left_motor(constrain(SPEED + consigne,0,1));
    pi.right_motor(constrain(SPEED - consigne,0,1));

// fin suiveur de ligne 

    pi.locate(0,1);
    sprintf(message,"%.2f",distance());
    pi.print(message,strlen(message));    

    pi.locate(0,0);
    sprintf(message,"%.1f %d",fabsf(error),sensors[2]);
    pi.print(message,strlen(message));
    pi.locate(0,1);
    sprintf(message,"%d %d %d",compteur_ligne_gauche,compteur_croisements,compteur_ligne_droite);
    pi.print(message,strlen(message));
}

void ligne_a_droite(void){ // ligne détectée à droite uniquement
    compteur_ligne_droite++;
}

void ligne_a_gauche(void){ // ligne détectée à gauche uniquement
    compteur_ligne_gauche++;
}

void croisement(void){ // croisement de lignes détecté
    compteur_croisements++;
    if (compteur_croisements == 4)
    {
        perpandicular_turn(1);
    }
    if (compteur_croisements == 5)
    {
        perpandicular_turn(1);
    }
    if (compteur_croisements == 7)
    {
        perpandicular_turn(1);
    }
    if (compteur_croisements == 8)
    {
        perpandicular_turn(1);
    }
    if (compteur_croisements == 9)
    {
        perpandicular_turn(1);
    }
    if(compteur_croisements == 10)
    {
        perpandicular_turn(1);
    }
}

void fin_de_ligne(void){ // sortie de piste détectée
    compteur_fin_de_ligne++;
    if (compteur_fin_de_ligne == 1)
    {
        pi.stop();
        u_turn();
        pi.backward(0.2);
    }
    if (compteur_fin_de_ligne == 2)
    {
        pi.stop();
        while(1);
    }
    
}

void priorite_a_droite(void){
    
}

void stopped(void){ // robot stoppé par le bouton 
    compteur_ligne_droite = 0;
    compteur_ligne_gauche = 0;
    compteur_croisements = 0;
}

// ======================= Main =======================//

int main(){
    pc.printf("HAL init done, proceeding...\n");
    pi.reset(); 
    wait_ms(400);
    
    if (pi.battery() < 4.8){
        pi.cls();
        pi.locate(0,0);
        sprintf(message,"LOW BATT !");
        pi.print(message,strlen(message));
        wait_button_press();
    }

    pi.cls();
    pi.locate(0,0);
    sprintf(message,"Calibrate");
    pi.print(message,strlen(message));
    
    wait_button_press();

    pi.sensor_auto_calibrate();

    pi.cls();
    pi.locate(0,0);
    sprintf(message,"Rdy");
    pi.print(message,strlen(message));
    //l

    setup();

    pc.printf("User init done, proceeding...\n");

    while(true){
        pi.calibrated_sensors(sensors);
        line_pos = pi.line_position();

        if(distance() < SEUIL_PRIORITE){
            priorite_a_droite();
        }

        loop();
        
        if(LINE_LEFT && !line_right && !line_left && !crossing){
            line_left = true;
        }
        if(LINE_RIGHT && !line_left && !line_right && !crossing){
            line_right = true;
        }

        if(LINE_RIGHT && line_left){
            crossing = true;
            line_left = false;
        }
        if(LINE_LEFT && line_right){
            crossing = true;
            line_right = false;
        }

        if (!LINE_LEFT && !LINE_RIGHT && (line_right || line_left || crossing)){
            if(line_right) ligne_a_droite();
            else if (line_left) ligne_a_gauche();
            else croisement();

            line_right = false, line_left = false, crossing = false;
        }

/*
        if(LINE_LEFT){
            if(!line_left){
                line_left = true;
                ligne_a_gauche();
            }
        } else if (line_left) line_left = false;

        if(LINE_RIGHT){
            if(!line_right){
                line_right = true;
                ligne_a_droite();
            }
        } else if (line_right) line_right = false;

        if(LINE_RIGHT && LINE_LEFT){
            if(!crossing){
                crossing = true;
                croisement();
            }
        } else if (crossing) crossing = false;
*/

         if(!ON_LINE){
            if(!lost_line){
                lost_line = true;
                fin_de_ligne();
            }
        } else if (lost_line) lost_line = false;

        if(!BTN){
            pi.stop();
            stopped();
            while(!BTN);

            wait_button_press();
            pi.reset(); 
            wait_ms(300);
            pi.sensor_auto_calibrate();

            wait_button_press();
        }

        pc.printf("%d %d %d %d %d online %d left %d \n", sensors[0],sensors[1],sensors[2],sensors[3],sensors[4],ON_LINE,LINE_LEFT);
    }
}