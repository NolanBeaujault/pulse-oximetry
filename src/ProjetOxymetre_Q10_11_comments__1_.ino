/*
Objectif : 
  Ce programme interagit avec le capteur MAX30101 pour détecter les battements cardiaques et mesurer le taux d'oxygène dans le sang (SpO2).
  On utilise un timer pour lire les données du capteur à intervalles réguliers ainsi que des filtres pour traiter les signaux.

  Fonctions utilisées :
  - setup() : Initialise la communication série et I2C, configure le capteur et démarre le timer.
  - loop() : Boucle principale qui traite les données du capteur lors des interruptions du timer.
  - TimerHandler() : Gère les interruptions du timer.
  - traiterCapteur() : Lit les données du capteur et calcule les moyennes.
  - LedV_pulse() : Gère l'allumage de la LED du shield en fonction des battements détectés.
  - resetFIFO() : Réinitialise le FIFO du capteur.
  - lireFIFO_6octets() : Lit 6 octets de données du FIFO.
  - LedR_ON() et LedIR_ON() : Allument les LEDs rouge et infrarouge.
  - ModeConfig() et ModeSpO2() : Configurent le mode du capteur.
  - lireRegistreMAX30101() : Lit un registre du capteur.
  - lirePartID() : Lit le PART ID du capteur.
  - beginTimer() : Configure et démarre un timer.
  - updateAmplitudeIR() et updateAmplitudeR() : Mettent à jour les amplitudes des signaux IR et rouge.
  - calculAmplitudeIR() et calculAmplitudeR() : Calculent les amplitudes des signaux IR et rouge.
  - calculRatio() : Calcule le ratio entre les amplitudes des signaux rouge et IR.
  - calculSpO2() : Calcule le taux d'oxygène dans le sang (SpO2) à partir du ratio.

  Composants :
  - Capteur MAX30101 connecté via bus I2C à l’adresse 0x57.
  - Led verte sur la pin 10 pour indiquer les battements cardiaques.
*/

#include <Wire.h> // iclure la bibliothèque pour I2C
#include "FspTimer.h" // Bibliothèque pour la gestion des timers FSP 

#define LedVPin 10
int seuil_h = 5; // seuil haut pour la détection des battements
int seuil_l = -80; // seuil bas pour la détection des battements

bool battement_detecte = false; // variable pour savoir si un battement a été détecté

const long interval_temps = 10;
unsigned long previousMillis = 0;

unsigned long moyenne_R_last = 0; // Dernière moyenne de la LED rouge
unsigned long moyenne_IR_last = 0; // Dernière moyenne de la LED infrarouge

unsigned long moyenne_IR = 0; // Moyenne actuelle de la LED infrarouge (Signal complet)
unsigned long moyenne_R  = 0; // Moyenne actuelle de la LED rouge (Signal complet)
unsigned long moyenne5s_R = 0; // Moyenne sur 5 secondes de la LED rouge (valeur DC)
unsigned long moyenne5s_IR = 0; // Moyenne sur 5 secondes de la LED infrarouge (valeur DC)

unsigned long compteur = 0; // Compteur pour le calcul des moyennes
unsigned long sommeR = 0; // Somme des valeurs de la LED rouge
unsigned long sommeIR = 0; // Somme des valeurs de la LED infrarouge

double AC_R = 0; // Composante AC de la LED rouge
double AC_R_last = 0; // Dernière composante AC de la LED rouge
double AC_R_filtre = 0; // Composante AC filtrée de la LED rouge
double AC_R_filtre_last = 0; // Dernière composante AC filtrée de la LED rouge

double AC_IR = 0; // Composante AC de la LED infrarouge
double AC_IR_last = 0; // Dernière composante AC de la LED infrarouge
double AC_IR_filtre = 0; // Composante AC filtrée de la LED infrarouge
double AC_IR_filtre_last = 0; // Dernière composante AC filtrée de la LED infrarouge

double alpha = 0.05; // Coefficient de filtrage
double alpha_pb = 0.5; // Coefficient de filtrage passe-bas

const int nombre_echantillons = 10; // Nombre d'échantillons à lire
byte donnees[6]; // Tableau pour stocker les données lues
byte FIFO_Conf_Data = 0b00000000; // Configuration du FIFO

int compteur_bpm = 0; // Compteur de battements par minute
const unsigned long interval_bpm = 15000;  // Intervalle de 15 secondes en ms
unsigned long previousMillis_bpm = 0; // Dernière mise à jour du compteur BPM

unsigned long total_IR = 0; // Total des valeurs de la LED infrarouge
unsigned long total_R = 0; // Total des valeurs de la LED rouge
unsigned long valeur_IR = 0; // Valeur actuelle de la LED infrarouge
unsigned long valeur_R = 0; // Valeur actuelle de la LED rouge

FspTimer audio_timer; // Instance du timer FSP

byte ADRESSE_I2C_MAX30101 = 0x57; // Adresse I2C du capteur MAX30101
byte REGISTRE_PART_ID = 0xFF;     // Adresse du registre contenant l’identifiant du capteur
byte part_id_voulu = 0x15;        // Valeur attendue pour identifier le capteur

byte address_LEDR = 0x0C;          // Registre de contrôle de la LED rouge
byte address_LED_IR = 0x0D;       // Registre de contrôle de la LED infrarouge

byte address_ModeConfig = 0x09;         // Registre de configuration du mode de fonctionnement
byte address_Mode_SpO2 = 0x0A;          // Registre de configuration SpO2

byte address_FIFO_data = 0x07;          // Registre de lecture des données FIFO

byte address_FIFO_WR_PTR = 0x04;        // Pointeur d’écriture FIFO
byte address_OVF_COUNTER = 0x05;        // Compteur de dépassement FIFO
byte address_FIFO_RD_PTR = 0x06;        // Pointeur de lecture FIFO

volatile bool flag_timer = false;

double periodMax_IR = 20;  // Valeur maximale de l'amplitude IR (par défaut)
double periodMin_IR = -20; // Valeur minimale de l'amplitude IR (par défaut)
double periodMax_R = 20;   // Valeur maximale de l'amplitude R (par défaut)
double periodMin_R = -20;  // Valeur minimale de l'amplitude R (par défaut)

void setup() {
  pinMode(LedVPin, OUTPUT);  // Initialisation de la pin de la LED verte
  Serial.begin(115200); // Démarrer la communication série à 115200 bauds pour affichage des résultats
  delay(1000);        // Attente pour que le moniteur série ait le temps de s'initialiser
  Wire.begin();       // Initialisation du bus I2C

  byte partID = lirePartID(); // Lire le PART ID du capteur

  Serial.print("Part ID : 0x");
  Serial.println(partID, HEX); // Affichage du PART ID en hexadécimal

  // Vérification si le PART ID correspond à celui attendu
  if (partID == part_id_voulu) {
    Serial.println("Capteur MAX30101 détecté avec succès !");
  } else {
    Serial.println("Erreur : Mauvais PART ID ou capteur non détecté.");
  }

  // Configuration du capteur
  ModeConfig();
  ModeSpO2();

  // Activation des LEDs
  LedR_ON();
  LedIR_ON();

  // Reset des registres FIFO
  resetFIFO();

  beginTimer(100); // démarrage du timer à 100 Hz
}

void loop() {
  // Gestion du timer
  // Permet de réinitialiser le compteur de battements toutes les 15 secondes
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis_bpm >= interval_bpm) {
    previousMillis_bpm = currentMillis;
    compteur_bpm = 0; // Réinitialisation
  }

  // On déclenche traiterCapteur() lors de l'interruption timer
  // Lors de l'interruption, flag passe à true, puis on le met à false
  if (flag_timer) {
    flag_timer = false;
    traiterCapteur();
    resetFIFO();

    sommeR += moyenne_R_last; // Ajout de la dernière moyenne rouge à la somme
    sommeIR += moyenne_IR_last; // Ajout de la dernière moyenne infrarouge à la somme
    compteur++; // Incrémentation du compteur

    if (compteur >= 512){ // Si le compteur atteint 512 (5 secondes à 100 Hz) 
      moyenne5s_R = sommeR/compteur; // Calcul de la moyenne sur 5 secondes (pour obtenir la valeur DC du rouge)
      moyenne5s_IR = sommeIR/compteur; // Calcul de la moyenne sur 5 secondes (pour obtenir la valeur DC de l'infrarouge)

      // Réinitialisation des valeurs pour le prochain calcul
      sommeR = 0; 
      sommeIR = 0;
      compteur = 0;
      }
  }
  // Filtrage des valeurs AC_R et AC_IR
  AC_R = AC_R_last + ((double)moyenne_R - (double)moyenne_R_last) - alpha * AC_R_last;
  AC_IR = AC_IR_last + ((double)moyenne_IR - (double)moyenne_IR_last) - alpha * AC_IR_last;
 
  AC_R_filtre = alpha_pb * (double)AC_R + (1-alpha_pb)* AC_R_filtre_last;
  AC_IR_filtre = alpha_pb * (double)AC_IR + (1-alpha_pb)* AC_IR_filtre_last;

  // Calcul des valeurs de DC_R et DC_IR
  moyenne_R_last = moyenne_R;
  moyenne_IR_last = moyenne_IR;


  AC_R_last = AC_R;
  AC_R_filtre_last = AC_R_filtre;
  AC_IR_last = AC_IR;
  AC_IR_filtre_last = AC_IR_filtre;

  // appelle la fonction pour gérer l'allumage de la LED verte
  LedV_pulse();

  // amplitude des signaux
  updateAmplitudeIR(AC_IR_filtre);
  updateAmplitudeR(AC_R_filtre);

  delay(2);
}

void TimerHandler(timer_callback_args_t __attribute((unused)) *p_args){
  flag_timer = true; // pour déclencher dans la loop
  /* cette fonction est appelée automatiquement lors du débordement du timer,
  elle contient juste le flag car on ne pouvait pas mettre de Serial.print pour 
  débugger (bug provoqué par un conflit de timer) donc TimerHandler s'occupe 
  juste de déclencher l'interruption et le reste se fait dans la loop avec traiterCapteur */
}

// Fonction pour traiter les données du capteur
void traiterCapteur(){
  total_IR = 0 ;
  total_R = 0;

  for (int i = 0; i < nombre_echantillons; i++) {

    lireFIFO_6octets(donnees);

    valeur_IR = (((unsigned long)donnees[0] << 16) | ((unsigned long)donnees[1] << 8) | (unsigned long)donnees[2]) & 0x3FFFFUL;
    valeur_R  = (((unsigned long)donnees[3] << 16) | ((unsigned long)donnees[4] << 8) | (unsigned long)donnees[5]) & 0x3FFFFUL;

    total_IR += valeur_IR;
    total_R  += valeur_R;
  }

  moyenne_IR = total_IR / nombre_echantillons; // on fait la moyenne sur le nombre d'échantillons afin d'avoir une valeur cohérente
  moyenne_R  = total_R / nombre_echantillons;  
}

// Fonction pour réinitialiser les valeurs d'amplitude pour IR et R
void resetAmplitude_IR(){
  periodMin_IR = -20;
  periodMax_IR = 20;
}
void resetAmplitude_R(){
  periodMin_R = -20;
  periodMax_R = 20;
}

// Fonction pour mettre à jour les valeurs d'amplitude pour IR et R sur une période (calcul effectué avec valeur DC toute les 5 secondes et valeur AC à chaque échantillon)
void updateAmplitudeIR(double valeur){
  if (valeur < periodMin_IR) periodMin_IR = valeur; // permet de détecter la valeur minimale sur une période
  if (valeur > periodMax_IR) periodMax_IR = valeur; // permet de détecter la valeur maximale sur une période
}
void updateAmplitudeR(double valeur){
  if (valeur < periodMin_R) periodMin_R = valeur;
  if (valeur > periodMax_R) periodMax_R = valeur;
}

// Fonction pour calculer l'amplitude pour IR et R
double calculAmplitudeIR(){
  double amplitudeIR = periodMax_IR - periodMin_IR; // quand on a un max et un min sur une période, on fait la différence entre la valeur max et min
  resetAmplitude_IR();
  return amplitudeIR;
}
double calculAmplitudeR(){
  double amplitudeR = periodMax_R - periodMin_R;
  resetAmplitude_R();
  return amplitudeR;
}

// Fonction pour calculer le ratio entre les amplitudes IR et R (d'après la formule de SpO2)
double calculRatio(double amplitudeR, double amplitudeIR, double dcR, double dcIR) {
  if (dcR <= 0 || dcIR <= 0) return -1.0;
  return (amplitudeR / dcR) / (amplitudeIR / dcIR);
}
double calculSpO2(double ratio) {
  if (ratio < 0) return -1.0;
  return 156.0 - 20.0 * ratio - 0.375 * (ratio * ratio);
}


// Fonction pour gérer l'allumage de la LED verte en fonction des battements détectés
void LedV_pulse() {
  //Si on a pas déjà detecté un battement et qu'on dépasse le seuil haut on allume la LED
  if (!battement_detecte && AC_IR_filtre > seuil_h) {

    // Si un battement est détecté, on allume la LED verte et on calcule le SpO2
    // On calcule les amplitudes et le ratio
    double amplitudeIR = calculAmplitudeIR();
    double amplitudeR = calculAmplitudeR();

    // calcul du ratio et du SpO2
    // vu que le ration est calculé avec les valeurs DC, la première valeur qu'on aura sera au bout de 5 secondes
    double R    = calculRatio(amplitudeR, amplitudeIR, moyenne5s_R, moyenne5s_IR);
    double spo2 = calculSpO2(R);

    Serial.print("SpO2 = ");
    Serial.print(spo2, 2);
    Serial.println("%");

    digitalWrite(LedVPin, HIGH);
    compteur_bpm++;
    battement_detecte = true; // on a detecté un battement
  }
  // Si on a déjà detecté un battement et qu'on passe sous le seuil bas on l'éteint
  else if (battement_detecte && AC_IR_filtre < seuil_l) {
    digitalWrite(LedVPin, LOW);
    battement_detecte = false;
  }
}

// Réinitialise les pointeurs FIFO à 0 et configure le registre FIFO
void resetFIFO() {
  byte init = 0x0;

  Wire.beginTransmission(ADRESSE_I2C_MAX30101);
  Wire.write(address_FIFO_WR_PTR);
  Wire.write(init);
  Wire.endTransmission();

  Wire.beginTransmission(ADRESSE_I2C_MAX30101);
  Wire.write(address_OVF_COUNTER);
  Wire.write(init);
  Wire.endTransmission();

  Wire.beginTransmission(ADRESSE_I2C_MAX30101);
  Wire.write(address_FIFO_RD_PTR);
  Wire.write(init);
  Wire.endTransmission();
}

// Lit 6 octets depuis le registre FIFO 
void lireFIFO_6octets(byte *donnees) {
  Wire.beginTransmission(ADRESSE_I2C_MAX30101);
  Wire.write(address_FIFO_data);
  Wire.endTransmission(false); // Reprise sans libérer le bus

  Wire.requestFrom(ADRESSE_I2C_MAX30101, 6);
  for (int i = 0; i < 6; i++) {
    if (Wire.available()) {
      donnees[i] = Wire.read(); // Remplissage du tableau
    }
  }
}

// Fonction pour activer la LED rouge du MAX30101 avec une intensité fixe (0x7F)
void LedR_ON() {
  Wire.beginTransmission(ADRESSE_I2C_MAX30101);
  Wire.write(address_LEDR);
  Wire.write(0x7F); // Intensité définie par la doc 
  byte erreur = Wire.endTransmission();

  if (erreur != 0) {
    Serial.print("Erreur LED Rouge : ");
    Serial.println(erreur);
  }
  delay(5);
}

// Fonction pour activer la LED infrarouge du MAX30101 avec une intensité fixe (0x7F)
void LedIR_ON(){
  Wire.beginTransmission(ADRESSE_I2C_MAX30101);
  Wire.write(address_LED_IR);
  Wire.write(0x7F); // Intensité définie par la doc
  byte erreur = Wire.endTransmission();

  if (erreur != 0) {
    Serial.print("Erreur LED IR : ");
    Serial.println(erreur);
  }
  delay(5);
}

// Fonction pour configurer le capteur en mode oxymétrie 
void ModeConfig() {
  byte dataConfig = 0x03; // 0x03 = mode SpO2 selon documentation

  Wire.beginTransmission(ADRESSE_I2C_MAX30101);
  Wire.write(address_ModeConfig); // Registre de configuration du mode
  Wire.write(dataConfig);        // Données à écrire (mode voulu)
  byte erreur = Wire.endTransmission();

  if (erreur != 0) {
    Serial.print("Erreur lors de l'écriture. Code : ");
    Serial.println(erreur);
  } else {
    Serial.println("Écriture réussie.");
  }

  delay(10); // Délai après configuration
}

// Fonction pour configurer le capteur en mode SpO2 avec fréquence et résolution 
void ModeSpO2() {
  byte dataSp02 = 0x75;  // 0111 0101 : fréquence et résolution 16 bits

  Wire.beginTransmission(ADRESSE_I2C_MAX30101);
  Wire.write(address_Mode_SpO2);
  Wire.write(dataSp02);
  byte erreur = Wire.endTransmission();

  if (erreur != 0) {
    Serial.print("Erreur lors de l'écriture. Code : ");
    Serial.println(erreur);
  } else {
    Serial.println("Écriture réussie.");
  }
  delay(10);
}

// Fonction pour lire un registre donné du capteur MAX30101
byte lireRegistreMAX30101(byte adresseRegistre) {
  Wire.beginTransmission(ADRESSE_I2C_MAX30101); // Début de la communication avec le capteur
  Wire.write(adresseRegistre);                  // Envoi de l'adresse du registre à lire
  byte erreur = Wire.endTransmission(false);    // Fin de la transmission sans libérer le bus

  if (erreur != 0) {
    // Si une erreur est survenue pendant la transmission
    Serial.print("Erreur de transmission I2C, code erreur : ");
    Serial.println(erreur);
    return 0xFF;  // Code d'erreur 
  }

  Wire.requestFrom(ADRESSE_I2C_MAX30101, (byte)1); // Demande d’un octet depuis le capteur
  if (Wire.available()) {
    return Wire.read(); // Si un octet est disponible, on le retourne
  } else {
    Serial.println("Aucune donnée disponible (capteur non détecté ?)");
    return 0xFF; // Valeur d’erreur si aucune donnée reçue
  }
}

// Fonction dédiée à la lecture du PART ID
byte lirePartID() {
  return lireRegistreMAX30101(REGISTRE_PART_ID);
}

bool beginTimer(float rate) {
  uint8_t timer_type = GPT_TIMER;
  int8_t tindex = FspTimer::get_available_timer(timer_type);
  if (tindex < 0){
    tindex = FspTimer::get_available_timer(timer_type, true);// Cherche un timer même réservé PWM
  }
  if (tindex < 0){
    return false;// Aucun timer disponible
  }

  // Force l'utilisation de timers normalement réservés à la PWM
  //FspTimer::force_use_of_pwm_reserved_timer();

  // Configure le timer en mode périodique à la fréquence demandée (rate)
  if(!audio_timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, rate, 0.0f, TimerHandler)){
    return false;
  }

  // Active l'interruption sur overflow (débordement)
  if (!audio_timer.setup_overflow_irq()){
    return false;
  }
  
  // Ouvre et démarre le timer
  if (!audio_timer.open()){
    return false;
  }

  if (!audio_timer.start()){
    return false;
  }
  return true;
}

/* Le code utilisé pour la gestion des interruption sur timer
a été tiré du code de Phil Schatzmann car nous n'avons pas trouvé de
bibliothèque fournissant des fonctions de haut niveau pour gerer les timers
pour les arduino wifi R4.
https://www.pschatzmann.ch/home/2023/07/01/under-the-hood-arduino-uno-r4-timers/
*/
