/*
  Objectif :
  Ce programme permet de contrôler l’état d’une LED verte à l’aide d’un bouton poussoir.
  À chaque pression du bouton, la LED change d’état : allumée si elle était éteinte, et inversement.

  Fonctions utilisées :
  - setup() : Initialise les broches et la communication série.
  - loop() : Gère la lecture du bouton et le changement d’état de la LED avec anti-rebond.

  Composants connectés :
  - LED verte connectée à la broche 10
  - Bouton poussoir connecté à la broche 9
*/

#define LedVPin 10   // Définition de la broche connectée à la LED verte
#define ButtonPin 9  // Définition de la broche connectée au bouton poussoir

bool EtatLed = 0;    // Variable représentant l'état actuel de la LED (0 = éteinte, 1 = allumée)

void setup() {
  pinMode(LedVPin, OUTPUT);    // Définir la broche de la LED comme une sortie
  pinMode(ButtonPin, INPUT);   // Définir la broche du bouton comme une entrée
  Serial.begin(9600);          // Initialiser la communication série à 9600 bauds pour debug
}

void loop() {
  int ButtonVal = digitalRead(ButtonPin);  // Lire l'état du bouton (0 ou 1)

  // Si le bouton est appuyé (1) et que la LED est actuellement éteinte
  if(ButtonVal == 1 && EtatLed == 0){
    digitalWrite(LedVPin, HIGH);  // Allumer la LED
    EtatLed = 1;                  // Mettre à jour l'état de la LED
    delay(100);                   // Anti-rebond : attendre un court instant
  }

  // Si le bouton est appuyé et que la LED est actuellement allumée
  else if(ButtonVal == 1 && EtatLed == 1){
    digitalWrite(LedVPin, LOW);   // Éteindre la LED
    EtatLed = 0;                  // Mettre à jour l'état de la LED
    delay(100);                   // Anti-rebond
  }
}
