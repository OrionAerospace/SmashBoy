11/2020

Todo o sistema passou por uma grande mudança, todo o código foi reescrito e toda sua lógica reformulada.

'Fluxograma RTOS V3' trás uma reestruturação das tasks, onde agora apenas uma única task pode acessar o 
 módulo de rádio, isso reduziu drasticamente a complexidade além da redução de memória e probabilidade
 de falhas/erros.
 
Outra grande mudança foi a lógica de "Maquina de estados" que simplificou drasticamente a programação e
 tornou o código mais simples de entender e mais seguro.




-----------------------------------------------------------
09/2020

[DESATUALIZADO] SmashBoy - FreeRTOS - Funcionamento Superficial Software [DESATUALIZADO] 
https://docs.google.com/spreadsheets/d/1H0UxWHC4-2a8tMSLyNS2wVXkgVRTFNAkbJkslC1BT0Q/edit#gid=0

[DESATUALIZADO][DESATUALIZADO]
O fluxograma RTOS V1 descreve a maneira como o sistema operacional do CubeSat irá se comportar, cada Task 
 pode ser compreendida como um conjunto de instruções que funciona em seu próprio loop de trabalho, 
 algumas tasks possuem dependências com outras.

[DESATUALIZADO] [DESATUALIZADO] [DESATUALIZADO] 
O rádio se encontrará na maior parte do tempo em RX mode, assim que uma nova mensagem chegar, será 
 encaminhada para a Task 0 (que estará ativa durante todo o tempo) esta task é responsável por fazer 
 uma triagem da informação e ativar a task que corresponde à tarefa, além disso, assim que uma mensagem 
 for recebida com sucesso, um ACK será emitido, informando para a base que o CubeSat recebeu o comando 
 com sucesso. A task 1 é responsável por concentrar toda a informação e envia-la ao transmissor 
 periodicamente. A task 2 é responsável por realizar o envio da imagem e é a única que não depende da 
 task 1 para o envio dos dados, a imagem será enviada assim que o comando for recebido. As demais tasks 
 são autoexplicativas.
 [DESATUALIZADO] [DESATUALIZADO] [DESATUALIZADO] 
