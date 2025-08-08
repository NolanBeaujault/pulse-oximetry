/*
  Objectif :
  Ce programme initialise le capteur MAX30101 en mode "oxymétrie" et active sa LED rouge.
  Il permet de tester si le capteur est bien initialisé et si l’intensité de la LED rouge peut être réglée.

  Fonctions utilisées :
  - setup() : Initialise la communication I2C et série, configure le mode du capteur et allume la LED rouge.
  - loop() : Vide, aucune action répétée.
  - LedR_ON() : Écrit dans le registre d’intensité de la LED rouge pour l’activer.
  - ModeConfig() : Configure le capteur pour fonctionner en mode oxymétrie.
  - lireRegistreMAX30101() : Lit une valeur dans un registre donné via I2C.
  - lirePartID() : Lit et retourne l'identifiant du capteur pour vérification (non utilisé ici mais présent pour debug).

  Composants :
  - Capteur MAX30101 connecté via bus I2C à l’adresse 0x57.
*/

#include <Wire.h> // Bibliothèque pour la communication I2C

// Déclarations des constantes et adresses de registres
byte ADRESSE_I2C_MAX30101 = 0x57; // Adresse I2C du capteur MAX30101
byte REGISTRE_PART_ID = 0xFF;     // Adresse du registre contenant l’identifiant du capteur
byte part_id_voulu = 0x15;        // Valeur attendue pour identifier le capteur

byte address_LEDR = 0x0C;          // Registre de contrôle de la LED rouge
byte address_ModeConfig = 0x09;    // Registre de configuration du mode de fonctionnement

void setup() {
  Serial.begin(9600); // Démarrer la communication série à 9600 bauds pour affichage des résultats
  delay(1000);        // Attente pour que le moniteur série ait le temps de s'initialiser
  Wire.begin();       // Initialisation du bus I2C

  ModeConfig(); // Configuration du capteur en mode oxygénation
  LedR_ON();    // Activation de la LED rouge
}

void loop() {
  // Aucune action répétée ici
}

// Fonction pour activer la LED rouge du MAX30101 avec une intensité fixe (0x7F)
void LedR_ON() {
  Wire.beginTransmission(ADRESSE_I2C_MAX30101); // Début de la transmission I2C
  Wire.write(address_LEDR);                      // Spécification du registre LED rouge
  Wire.write(0x7F);                             // Valeur d’intensité (mi-intensité d’après la doc)
  byte erreur = Wire.endTransmission();         // Fin de transmission + retour du code erreur

  if (erreur != 0) {
    Serial.print("Erreur lors de l'écriture. Code : ");
    Serial.println(erreur);
  } else {
    Serial.println("Écriture réussie.");
  }

  delay(10); // Petit délai après l’écriture

  // Lecture du registre pour vérification (debug)
  byte lu = lireRegistreMAX30101(adress_LEDR);
  Serial.print("Valeur lue après écriture : 0x");
  Serial.println(lu, HEX);
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
