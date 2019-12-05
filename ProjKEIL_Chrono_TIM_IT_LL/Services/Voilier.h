//Configurer le moteur
void MoteurConf(void);

//Configurer la t�l�commande
void RxTelecommandeConf(void);

//void AlimentationConfig(void);

//Configurer le sens de rotation du plateau
void SensPlateau(int sens);

//Configurer la vitesse de rotation
void VitessePlateau(int pr);

//Liaison entre la t�lecommande et le moteur
void Moteur_telecommande (void);

// Configurer la girouette
void GirouetteConfig(void);

// Configurer du Servo Voile
void ServoConfig(void);

void InitAngleGirouette(void);
// R�cup�rer de l'angle de la girouette
int GetAngleGirouette(void);

// Configurer le pulse du servo
void SetPulseServo(void);

//Configurer l'accelero
void AcceleroConfig(void);


int Conversion(void);
