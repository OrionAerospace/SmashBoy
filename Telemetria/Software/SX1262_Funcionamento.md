# SX1262

---------------------------

### Notas sobre o funcionamento do SX1262 e SX1280.

Henrique Terzi Lucchetta, 20/08/2020.

----------------------------

Este CI não necessita de biblioteca, pois o mesmo já implementa uma sequência de funcionamento semelhante ao uso de uma. É fundamental monitorar o pino BUSY, pois com base no nivel lógico deste pino que será possível determinar quando o CI está apto a receber um novo comando, caso este pino não seja monitorado, ocorrerão falhas inesperadas de funcionamento.

A interpretação do datasheet é confusa num primeiro momento, diferentemente de sua versão anterior (SX1272) este CI não fornece a tabela completa de registradores.

A seção 14 do datasheet explica a sequência de funcionamento para uma transmissão e para a recepção. Cada item descrito na seção 14 tem seu funcionamento descrito na seção 13, cada função da seção 13 possui um OpCode, esse OpCode é sempre o primeiro byte que vai informar para o transmissor o que os dados seguintes representam, exemplo:

SetStandby, coloca o CI em Standby e define qual o oscilador interno será utilizado durante este modo (RC ou XTAL), a mensagem que será transmitida deve primeiramente verificar se o pino BUSY está em LOW, caso esteja em HIGH aguarde passar para LOW, em seguida o pino SPI NSS deve ir para LOW para o SX1262 receber o comando, depois deve enviar o OpCode correspondente a SetStandby (no caso, 0x80) e o próximo byte correspondente ao oscilador utilizado (StdbyConfig), neste caso pode ser (0x00 ou 0x01), a mensagem completa enviada via SPI será então **0x80 0x00** ou **0x80 0x01**.

Read/WriteRegister e Read/WriteBuffer, também possuem OpCodes, endereços e offsets que devem ser enviados antes do valor a ser lido ou inserido nos registradores, exemplo WriteRegister:

 > BUSY == LOW
 
 > NSS = LOW
 
 > SPI_WRITE = **0x0D 0x07 0x40 0x34 0x44**
 
 > NSS = HIGH
