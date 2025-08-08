/*
  Objectif :
  Ce programme interagit avec le capteur MAX30101 pour lire les données des LEDs rouge et infrarouge,
  et calcule les moyennes des valeurs lues. Il utilise un timer pour déclencher la lecture des données
  à intervalles réguliers.

  Fonctions utilisées :
  - setup() : Initialise la communication série et I2C, configure le capteur et démarre le timer.
  - loop() : Boucle principale qui traite les données du capteur lors des interruptions du timer.
  - TimerHandler() : Gère les interruptions du timer.
  - traiterCapteur() : Lit les données du capteur et calcule les moyennes.
  - resetFIFO() : Réinitialise le FIFO du capteur.
  - lireFIFO_6octets() : Lit 6 octets de données du FIFO.
  - LedR_ON() et LedIR_ON() : Allument les LEDs rouge et infrarouge.
  - ModeConfig() et ModeSpO2() : Configurent le mode du capteur.
  - lireRegistreMAX30101() : Lit un registre du capteur.
  - lirePartID() : Lit le PART ID du capteur.
  - beginTimer() : Configure et démarre un timer.

  Composants :
  - Capteur MAX30101 connecté via bus I2C à l’adresse 0x57.
*/

#include <Wire.h> // Bibliothèque pour la communication I2C
#include "FspTimer.h" // Bibliothèque pour la gestion des timers FSP

// Déclarations des constantes et adresses de registres
const long interval_temps = 10;
unsigned long previousMillis = 0;

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

void setup() {
  Serial.begin(9600); // Démarrer la communication série à 9600 bauds pour affichage des résultats
  delay(1000);        // Attente pour que le moniteur série ait le temps de s'initialiser
  Wire.begin();       // Initialisation du bus I2C

  byte partID = lirePartID(); // Lire le PART ID du capteur

  Serial.print("Part ID : 0x");
  Serial.println(partID, HEX); // Affichage du PART ID en hexadécimal

  // Vérification si le PART ID correspond à celui attendu
  if (partID == PART_ID_ATTENDU) {
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

  beginTimer(100); // Démarrage du timer à 100 Hz
}

void loop() {
  // On déclenche traiterCapteur() lors de l'interruption timer
  // Lors de l'interruption, flag passe à true, puis on le met à false
  if (flag_timer) {
    flag_timer = false;
    traiterCapteur();
  }
}

void TimerHandler(timer_callback_args_t __attribute((unused)) *p_args) {
  flag_timer = true; // Pour déclencher dans la loop

  /* Cette fonction est appelée automatiquement lors du débordement du timer,
  elle contient juste le flag car on ne pouvait pas mettre de Serial.print pour
  débugger (bug provoqué par un conflit de timer) donc TimerHandler s'occupe
  juste de déclencher l'interruption et le reste se fait dans la loop avec traiterCapteur */
}

void traiterCapteur() {
  // Déclenchée par millis TimerHandler
  Serial.println(millis());  // Debug
  const uint8_t nombre_echantillons = 10;

  uint32_t total_IR = 0;
  uint32_t total_R = 0;

  for (uint8_t i = 0; i < nombre_echantillons; i++) {
    byte donnees[6];
    lireFIFO_6octets(donnees);

    uint32_t valeur_IR = (donnees[0] << 16) | (donnees[1] << 8) | donnees[2];
    uint32_t valeur_R  = (donnees[3] << 16) | (donnees[4] << 8) | donnees[5];

    total_IR += valeur_IR;
    total_R  += valeur_R;
  }

  uint32_t moyenne_IR = total_IR / nombre_echantillons;
  uint32_t moyenne_R  = total_R / nombre_echantillons;
  resetFIFO();

  Serial.println(moyenne_R);
}

// Réinitialise les pointeurs FIFO à 0
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
  if (tindex < 0) {
    tindex = FspTimer::get_available_timer(timer_type, true); // Cherche un timer même réservé PWM
  }
  if (tindex < 0) {
    return false; // Aucun timer disponible
  }

  // Force l'utilisation de timers normalement réservés à la PWM
  FspTimer::force_use_of_pwm_reserved_timer();

  // Configure le timer en mode périodique à la fréquence demandée (rate)
  if (!audio_timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, rate, 0.0f, TimerHandler)) {
    return false;
  }

  // Active l'interruption sur overflow (débordement)
  if (!audio_timer.setup_overflow_irq()) {
    return false;
  }

  // Ouvre et démarre le timer
  if (!audio_timer.open()) {
    return false;
  }

  if (!audio_timer.start()) {
    return false;
  }
  return true;
}

/* Le code utilisé pour la gestion des interruptions sur timer
a été tiré du code de Phil Schatzmann car nous n'avons pas trouvé de
bibliothèque fournissant des fonctions de haut niveau pour gérer les timers
pour les Arduino WiFi R4.
https://www.pschatzmann.ch/home/2023/07/01/under-the-hood-arduino-uno-r4-timers/
*/
