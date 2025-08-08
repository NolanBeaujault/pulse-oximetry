/*
  Objectif :
  Ce programme permet de détecter si un capteur MAX30101 est connecté au microcontrôleur via le bus I2C.
  Pour cela, il lit le registre `PART ID` du capteur et compare la valeur retournée à celle attendue.

  Fonctions utilisées :
  - setup() : Initialise la communication série et I2C, lit le PART ID et affiche un message selon le résultat.
  - loop() : Vide (aucune action répétée dans ce programme de test).
  - lireRegistreMAX30101() : Lit un registre du capteur MAX30101 via I2C.
  - lirePartID() : Spécifique à la lecture du registre `PART ID`.

  Composants connectés :
  - Capteur MAX30101 (oxymètre de pouls) via bus I2C (adresse : 0x57)
*/

#include <Wire.h> // Inclusion de la bibliothèque pour la communication I2C

const byte ADRESSE_I2C_MAX30101 = 0x57; // Adresse I2C du capteur MAX30101 (donnée par la documentation)
const byte REGISTRE_PART_ID = 0xFF;     // Adresse du registre contenant l’identifiant du capteur
const byte PART_ID_ATTENDU = 0x15;      // Valeur attendue du PART ID pour confirmer la présence du capteur

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
}

void loop() {
  // (détection au démarrage donc rien à faire ici)
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
