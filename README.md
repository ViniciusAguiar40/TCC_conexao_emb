# TCC Conexão Embarcada
Este repositório apresenta a parte do código referente à solução de conexão embarcada no robô com a rede. A plataforma utilizada para programação é o ESP-IDF 4.4.2, com o microcontrolador ESP32-PICO-MINI02U, da Espressif Systems.

## Pinagem
A tabela abaixo mostra os pinos utilizados e a sua respectiva função e nome (se aplicável).

| Pino | Função                                      | Nome do pino |
|------|---------------------------------------------|--------------|
|  12  |SPI Master Output Slave Input				 | GPIO_MOSI	|
|  13  |SPI Master Input Slave Output				 | GPIO_MISO	|
|  14  |SPI Software Clock							 | GPIO_SLCK	|
|  15  |SPI Chip Select								 | GPIO_CS		|

## Componentes
### Periph_spi
Trata da inicialização e tratamento do periférico que utiliza o protocolo SPI. O microcontrolador opera como escravo e recebe informações do STM32.
Esta componente faz o _parser_ das informações recebidas e utiliza da componente MQTT manager para publicação.

### Wifi_mgr
Trata da conexão do microcontrolador ao wi-fi. É necessária uma antena no hardware para que o sinal alcance um nível satisfatório. No arquivo .c da componente há duas definições que devem ser observadas. Elas são utilizadas para definição do SSID e a senha da rede a ser conectada.

```c
#define EXAMPLE_ESP_WIFI_SSID   "SSID"
#define EXAMPLE_ESP_WIFI_PASS   "SENHA"
```

### MQTT_mgr
Trata da conexão com o Broker MQTT e com a operação de _publish_. Vale ressaltar as definições do endereço e porta do broker, além das informações de usuário e senha para conexão estão especificadas no arquivo .c:

```c
#define CONFIG_BROKER_URL       "mqtt://mqtt.tago.io"
#define CONFIG_BROKER_PORT      (1883)
#define CONFIG_BROKER_USERNAME  "Usuario"
#define CONFIG_BROKER_PASSWORD  "Senha"
```

O broker utilizado é o próprio broker presente no [TagoIO](https://help.tago.io/portal/en/kb/articles/32-mqtt). A porta utilizada é a TCP/IP, sem criptografia.