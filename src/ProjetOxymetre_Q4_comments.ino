/*
   Objectif :
    - Vérifier la présence du capteur grâce au PART ID.
    - Configurer le capteur en mode SpO2 (oxygénation du sang).
    - Allumer la LED rouge.
    - Lire régulièrement les valeurs infrarouge (IR) et rouge (R) depuis la FIFO.
    - Afficher les données série pour traitement/visualisation.

  Fonctions principales :
    - setup() : Initialisation série, I2C, détection du capteur et configuration.
    - loop() : Lecture des données IR/R et affichage.
    - lirePartID() : Récupère l'identifiant du capteur.
    - lireRegistreMAX30101() : Lecture d’un registre I2C donné.
    - ModeConfig(), ModeSpO2() : Configuration du mode du capteur.
    - LedR_ON(), LedIR_ON() : Contrôle des LEDs.
    - lireFIFO_6octets() : Lecture des données FIFO (6 octets).
    - resetFIFO() : Réinitialisation des pointeurs FIFO.
*/

#include <Wire.h> // Bibliothèque pour la communication I2C

// Déclarations des constantes et adresses de registres
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

  // Activation de la LED rouge
  LedR_ON();

  // Lecture initiale FIFO
  lecture_FIFO();
}

// Boucle principale 
void loop() {
  byte donnes[6];                   // Tableau pour stocker les 6 octets FIFO
  lireFIFO_6octets(donnes);        // Lecture FIFO
  resetFIFO();                     // Réinitialisation des pointeurs FIFO

  // Reconstruction des données sur 3 octets pour IR et R
  uint32_t valeur_IR = (donnes[0] << 16) + (donnes[1] << 8) + donnes[2];
  uint32_t valeur_R = (donnes[3] << 16) + (donnes[4] << 8) + donnes[5];

  // Affichage des valeurs IR et rouge
  Serial.print("Valeur IR: ");
  Serial.print(valeur_IR);
  Serial.print(" Valeur R: ");
  Serial.println(valeur_R);
}

//  Réinitialise les pointeurs FIFO à 0 
void resetFIFO() {
  byte init = 0x00;

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
  delay(10);
}

// Fonction pour activer la LED infrarouge du MAX30101 avec une intensité fixe (0x7F)
void LedIR_ON() {
  Wire.beginTransmission(ADRESSE_I2C_MAX30101);
  Wire.write(address_LED_IR);
  Wire.write(0x7F); // Intensité définie par la doc
  byte erreur = Wire.endTransmission();

  if (erreur != 0) {
    Serial.print("Erreur LED IR : ");
    Serial.println(erreur);
  }
  delay(10);
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
