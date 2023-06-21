# FoxBaja-Eletronica-FX4.0

# Estrutura de código das ECU's de sensores

```
A cada rodar do loop principal, irá ler os dados dos sensores e enviar para uma fila, que enviará os dados para a ECU do painel.

o que vai ser necessário:

- task para ler os dados dos sensores
- fila para enviar os dados para a ECU do painel
- task para enviar os dados para a ECU do painel

```

# Estrutura de código da ECU do painel

```

IDEIA 1

A ECU do painel, dentro do loop principal, vai receber os dados constantemente. A cada dado recebido, irá identificar o id da mensagem (quem enviou), e então salvar em uma variável correspondente.
Com isso, vamos usar um semáforo que vai ser liberado 5x por segundo. Assim, 5x por segundo, a task vai pegar os últimos dados recebidos, e enviar para a task principal por meio de uma fila. A task principal, vai receber os dados, e atualizar o painel com os dados recebidos.

o que vai ser necessário:

- task auxiliar para ler os dados recebidos e salvar em variáveis
- semáforo para liberar a task auxiliar 5x por segundo
- fila para enviar os dados para a task principal
- task principal para receber os dados e atualizar o painel

OBS: Verificar se é necessário o uso de semáforo, pq com o auxílio de uma biblioteca natural da esp, ela automaticamente executa cada task em um tempo determinado, e não é necessário o uso de semáforo para isso.

```

```

IDEIA 2 (menos provável)

A cada loop do loop principal, ele enviará um pacote remoto para cada ECU. Cada uma, irá responder ao pacote com um pacote de dados, contendo os dados que devem ser atualizados no painel. A ECU do painel, irá receber os pacotes de dados, e irá atualizar o painel com os dados recebidos.

```
