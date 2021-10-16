#include <Stepper.h>

int stepsPerRevolution = 800;  // Aqui configurei micropassos 1/4 --- 
                               //precisa de 800 micropassos para uma revolução no Nema 16 Unipolar

const int fimdeCurso1 = 2;  // porta 2 do Arduino UNO
const int fimdeCurso2 = 5;  // porta 5 do Arduino UNO
const int pinoLedAzul = 3;
const int pinoLedVerde = 4;

bool volta = false;


// inicializa a biblioteca criando uma instancia do motor
Stepper myStepper(stepsPerRevolution, 8, 9); 

void setup() {
  pinMode(fimdeCurso1,INPUT_PULLUP);
  pinMode(fimdeCurso2,INPUT_PULLUP);
  pinMode(pinoLedAzul, OUTPUT); //DEFINE O PINO COMO SAÍDA
  pinMode(pinoLedVerde, OUTPUT); //DEFINE O PINO COMO SAÍDA
  
  myStepper.setSpeed(800); // ajusta a velocidade inicial para 800 passos
}

void loop() {
 
  stepsPerRevolution = 800;

  // faz a leitura do fim de curso
  
  if (digitalRead(fimdeCurso1) == HIGH && volta == false) {
          delay(1);
          digitalWrite(pinoLedAzul, HIGH); //ACENDE O LED
          digitalWrite(pinoLedVerde, LOW); //APAGA O LED
          myStepper.setSpeed(100); // ajusta a velocidade inicial para 100 passos
          myStepper.step(-stepsPerRevolution);
   } 
   
   if (digitalRead(fimdeCurso1) == LOW && volta == false) {
          myStepper.setSpeed(2); // ajusta a velocidade inicial para 2 passos 
          digitalWrite(pinoLedAzul, LOW); //APAGA O LED
          digitalWrite(pinoLedVerde, HIGH); //ACENDE O LED
          
          delay(2000); //PAUSA DE 2 SEGUNDOS
          volta = true;
    }

    if( volta == true){
       digitalWrite(pinoLedVerde, HIGH); //ACENDE O LED
       myStepper.setSpeed(200); // ajusta a velocidade inicial para 200 passos
       myStepper.step(stepsPerRevolution);   
    }

    if (digitalRead(fimdeCurso2) == HIGH && volta == true) {
          myStepper.setSpeed(2); // ajusta a velocidade inicial para 2 passos
          digitalWrite(pinoLedAzul, HIGH); //ACENDE O LED
          digitalWrite(pinoLedVerde, LOW); //APAGA O LED
          
          delay(2000); //PAUSA DE 2 SEGUNDOS
          volta = false;
    }
}