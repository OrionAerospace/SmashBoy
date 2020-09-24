O fluxograma RTOS descreve a maneira como o sistema operacional do CubeSat irá se comportar, cada Task 
 pode ser compreendida como um conjunto de instruções que funciona em seu próprio loop de trabalho, 
 algumas tasks possuem dependências com outras.

O rádio se encontrará na maior parte do tempo em RX mode, assim que uma nova mensagem chegar, será 
 encaminhada para a Task 0 (que estará ativa durante todo o tempo) esta task é responsável por fazer 
 uma triagem da informação e ativar a task que corresponde à tarefa, além disso, assim que uma mensagem 
 for recebida com sucesso, um ACK será emitido, informando para a base que o CubeSat recebeu o comando 
 com sucesso. A task 1 é responsável por concentrar toda a informação e envia-la ao transmissor 
 periodicamente. A task 2 é responsável por realizar o envio da imagem e é a única que não depende da 
 task 1 para o envio dos dados, a imagem será enviada assim que o comando for recebido. As demais tasks 
 são autoexplicativas.
 

(#ATUALIZAR#)
Algoritmo de deploy: Um comando chega por RF ao módulo de rádio que estará em modo de recebimento, 
 ocorre uma interrupção no microcontrolador (cubeSat), a leitura e triagem da informação. O sistema 
 operacional identifica que o comando é referente a abertura da antena e executa a task correspondente. 
 Um ACK (acknowledge) é retornado como feedback informando para a estação que o cubeSat entendeu o 
 comando, em seguida o elemento de aquecimento e um timer de segurança são acionados, caso o elemento 
 de aquecimento falhe e não libere a antena, o timer impede que ele continue funcionando por um tempo 
 indeterminado (é necessário haver uma forma de detectar que o deploy da antena teve sucesso, com um 
 switch por exemplo). Em seguida, ocorrendo o deploy ou o estouro do timer, uma mensagem de feedback 
 da situação é enviada para a base para informar o estado. Em caso de sucesso, a task é deletada para 
 liberar memória, em caso de falha, ela pode ser tentada novamente.
