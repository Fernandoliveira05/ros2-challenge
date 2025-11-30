# PONDERADA SEMANA 06 - Navegação e mapeamento (PARCIALMENTE GERADO POR INTELIGÊNCIA ARTIFICIAL)

## Assista ao vídeo:
- A imagem abaixo te redireciona para o vídeo explicativo do projeto:
<div align="center">
  <a href="https://www.youtube.com/watch?v=FdRqLgt-Rgo">
    <img src="https://external-preview.redd.it/when-will-we-see-robots-roaming-the-streets-v0-RJGaMxlF3hb9DsxQbTZHo3bI2HC0e161MwZ1opqAOBE.jpg?width=640&crop=smart&auto=webp&s=d063199d2f58575a3814ee09d8071a2030f0f5db" alt="Vídeo do Projeto ULA Completa">
  </a>
</div>

## O que são grafos?

&nbsp;&nbsp;Ao longo deste módulo, nos aprofundamos em grafos e sua relevância para a computação moderna. Grafos são amplamente utilizados porque seus fundamentos matemáticos vêm sendo desenvolvidos e estudados há muitos anos, muito antes sequer de existirem computadores. </br>
&nbsp;&nbsp;Esse amadurecimento teórico permitiu que, hoje, eles sejam aplicados em problemas complexos, desde navegação e mapeamento até redes sociais e recomendação de conteúdo. Um exemplo é o Facebook, que teve sua ascensão impulsionada pela adoção de estruturas de grafo no sistema de anúncios (Facebook Ads), que possibilitou uma segmentação mais inteligente, conexões mais precisas e um entendimento profundo das relações entre usuários.</br>
&nbsp;&nbsp;Os grafos nada mais são do que estruturas matemáticas compostas por vértices (ou nós) e arestas, que representam conexões entre esses vértices. Essa simplicidade estrutural é justamente o que torna os grafos extremamente poderosos e versáteis: eles conseguem modelar praticamente qualquer tipo de relação, fluxo ou estrutura de dependência.

## O que estamos fazendo aqui?

&nbsp;&nbsp;A proposta deste repositório é atender ao desafio da Ponderada Navegação e Mapeamento, proposto pelo professor Rodrigo Nicola. O objetivo central é desenvolver um algoritmo capaz de controlar um robô em um simulador criado com PyGame, utilizando comunicação via ROS2.

&nbsp;&nbsp;Para isso, o código deve ser implementado em uma linguagem sem garbage collector, então nada de Python! 

O projeto se divide em duas partes principais:

1. **Interpretação do mapa:** implementar um algoritmo que utiliza o protocolo fornecido para requisitar e obter o mapa do jogo.

2. **Navegação inteligente:** mapear o ambiente, processá-lo como grafo e buscar a melhor rota até o objetivo, aplicando algoritmos de pathfinding.

## Como fazemos isso? 

&nbsp;&nbsp;Desenvolvi um pacote ROS2 contendo dois códigos em C++, cada um implementando um dos algoritmos necessários para resolver os desafios da ponderada. Ambos os códigos estão disponíveis neste repositório e, mais adiante, descrevo exatamente como executá-los e quais passos seguir para reproduzir os resultados sem dificuldades. </br>
&nbsp;&nbsp;O primeiro algoritmo utiliza o serviço `/get_map` para obter a matriz do labirinto, converte esse mapa em uma representação matricial e, a partir disso, interpreta as relações entre os nós para permitir que o robô navegue até o objetivo final de maneira rápida e prática. Convenhamos que esse é mais fácil, porque já recebemos do serviço toda a estrutura do mapa. </br>
&nbsp;&nbsp;O segundo algoritmo é mais elaborado. Antes de buscar o alvo, o robô precisa explorar o mapa por completo: identificar regiões livres, registrar obstáculos, construir um grafo navegável e, somente depois disso, retornar ao ponto de origem. Com o mapa completo reconstruído, o sistema então aplica um algoritmo de caminho mínimo (BFS em grade com custo unitário, equivalente ao Dijkstra nesse contexto) para calcular a rota mais eficiente entre a posição inicial e o objetivo. 

---

## Estrutura do repositório

Este repositório já está organizado como um workspace ROS 2 (Jazzy), com os pacotes principais:

- `cg/`: simulador em PyGame (labirinto, robô, editor de mapas).
- `cg_interfaces/`: mensagens e serviços usados pelo jogo:
  - `RobotSensors.msg`
  - `GetMap.srv`
  - `MoveCmd.srv`
  - `Reset.srv`
- `cg_solver/`: **pacote com as soluções em C++ desenvolvidas por mim**.
- `cg_teleop/`: nó de teleoperação por teclado.

Os códigos principais que implementei estão em:

```text
src/cg_solver/src/
  ├── solver.cpp   # nó maze_solver (usa /get_map + BFS)
  └── mapear.cpp   # nó de mapeamento online + pathfinding
```

---

## Clonando o repositório

Assumindo que você já tem o ROS 2 Jazzy instalado:

```bash
# Escolha um diretório para o workspace
cd ~
git clone https://github.com/fernandoliveira05/ros2-challenge.git fernandoliveira05-ros2-challenge
cd fernandoliveira05-ros2-challenge
```

> Se você estiver avaliando o código a partir de um ZIP enviado, basta extrair o conteúdo e entrar na pasta `fernandoliveira05-ros2-challenge`.

---

## Pré‑requisitos

- **ROS 2 (A principio qualquer Distro)** instalado e configurado (Ubuntu 24.04 ou ambiente compatível). 
- `colcon` para build do workspace:
  ```bash
  sudo apt install python3-colcon-common-extensions
  ```
- Dependências padrão do ROS 2 para C++ (compilador, ament, etc.).

Antes de rodar qualquer comando, lembre de carregar o ambiente do ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

---

## Como instalar o ROS2 (Jazzy, pois foi o que utilizei)
(Ubuntu 24.04)

A ponderada utiliza ROS 2 para comunicação entre os nós do robô e o jogo.
Se você ainda não tem o ROS 2 instalado, siga este guia oficial simplificado para instalar o ROS 2 Jazzy LTS.

1. Atualize seu sistema
```bash
sudo apt update && sudo apt upgrade -y
```

2. Adicione os repositórios do ROS
```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update
```

3. Instale as chaves do ROS 2
```bash
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
```

4. Adicione o repositório do ROS 2 Jazzy
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

5. Atualize novamente
```bash
sudo apt update
```

6. Instale o ROS 2 Jazzy Desktop
```bash
sudo apt install ros-jazzy-desktop -y
```
Caso queira uma versão mais leve, tente:
```bash
sudo apt install ros-jazzy-ros-base -y
```

7. Instale o conjunto de ferramentas colcon
```bash
sudo apt install python3-colcon-common-extensions -y
```

8. Configure o ambiente
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

9. Teste a instalação
```bash
ros2 run demo_nodes_cpp talker
```

Se aparecer algo como:
```bash
Publishing: 'Hello World'
```

Tá lá, menininho! Deu certo!

## Build do workspace

Dentro da pasta do repositório:

```bash
cd fernandoliveira05-ros2-challenge

# (opcional, mas recomendado)
rm -rf build/ install/ log/

# Build dos pacotes
colcon build --symlink-install

# Carregar o ambiente do workspace
source install/setup.bash
```

A partir daqui, você já deve conseguir listar os pacotes e executáveis:

```bash
ros2 pkg list | grep cg
ros2 pkg executables cg_solver
```

---

## Onde estão e o que fazem os códigos em C++

### `solver.cpp` — nó **maze_solver** (usa `/get_map`)

Arquivo:  
```text
src/cg_solver/src/solver.cpp
```

Executável/nó instalado como:  
```bash
ros2 run cg_solver maze_solver
```

**Ideia geral da lógica**:

1. Chama o serviço `/get_map` (tipo `cg_interfaces/srv/GetMap`) e recebe:
   - `occupancy_grid_flattened`: lista linear de células (`"b"`, `"f"`, `"r"`, `"t"`…)
   - `occupancy_grid_shape`: `[height, width]`.
2. Reconstrói uma matriz 2D `grid[height][width]` a partir do vetor flatten:
   - `'b'` → parede (não atravessável)
   - `'f'` → célula livre
   - `'r'` → posição inicial do robô (**start**)
   - `'t'` → objetivo (**goal**)
3. A partir dessa matriz, roda um **BFS (Breadth-First Search)** em grade:
   - cada célula livre é um nó;
   - cada movimento para cima/baixo/esquerda/direita é uma aresta de custo 1;
   - como o custo é uniforme, o BFS encontra automaticamente o **caminho mais curto em número de passos**.
4. Reconstrói o caminho do `goal` até o `start` usando uma matriz de `parent`.
5. Converte o caminho em comandos de movimento:
   - diferença entre coordenadas vira `"up"`, `"down"`, `"left"`, `"right"`.
6. Para cada comando, chama o serviço `/move_command` (`cg_interfaces/srv/MoveCmd`), que faz o robô se mover no simulador.

Em resumo: **esse nó resolve o labirinto “de uma vez só”**, assumindo que o mapa já está totalmente conhecido via `/get_map`.

---

### `mapear.cpp` — nó de mapeamento online + caminho ótimo

Arquivo:  
```text
src/cg_solver/src/mapear.cpp
```

Executável/nó (nome sugerido):  
```bash
ros2 run cg_solver mapear
```

**Ideia geral da lógica**:

1. Assina o tópico de sensores (por exemplo, `/culling_games/robot_sensors`) usando a mensagem `cg_interfaces::msg::RobotSensors`.
2. A cada callback, o nó recebe o que existe ao redor do robô:
   - `up`, `down`, `left`, `right` (strings como `"b"`, `"f"`, `"r"`, `"t"`).
3. Tradução para o mapa lógico:
   - converte essas strings em um `CellState` (`Free`, `Wall`, `Goal`, `Unknown`);
   - atualiza uma estrutura `map_` que guarda o tipo de cada coordenada;
   - registra também a posição do objetivo (`goal_`) assim que ele é visto.
4. **Exploração com DFS**:
   - o robô anda apenas para células livres e ainda não visitadas;
   - uma pilha (`dfs_stack_`) guarda o caminho percorrido;
   - quando não há mais vizinhos livres novos, o robô faz **backtracking** voltando pela pilha;
   - esse processo continua até explorar todo o mapa alcançável e retornar ao ponto inicial (`start_`).
5. **Cálculo do melhor caminho**:
   - com o mapa completo em mãos, o código aplica BFS em cima do grafo implícito na grade;
   - como todos os movimentos custam 1, o BFS é equivalente ao Dijkstra com peso 1 em todas as arestas;
   - o resultado é o **caminho ótimo entre o start e o goal**.
6. **Execução do caminho ótimo**:
   - o caminho calculado é convertido novamente em direções `"up"`, `"down"`, `"left"`, `"right"`;
   - o nó passa a enviar comandos para `/move_command` seguindo essa rota até o objetivo.

Em resumo: **esse nó primeiro explora o ambiente, constrói o mapa, e só depois calcula e executa o caminho mais curto**.

---

## Como executar o simulador (jogo)

O simulador em PyGame está no pacote `cg`.  
Depois de ter feito o build e carregado o workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd fernandoliveira05-ros2-challenge
source install/setup.bash

# Executar o jogo (maze)
ros2 run cg maze
```

> Dependendo da configuração original do professor, outros executáveis podem existir (`edit`, `test`, etc.), mas para a ponderada o foco é o `maze`.

O jogo abrirá uma janela com o labirinto, e os nós em C++ irão interagir com ele via ROS 2.

---

## Executando os meus nós C++ individualmente

> **Importante:** sempre use dois terminais – um para o jogo e outro para o nó de solução.

### 1. Executar apenas o `maze_solver` (usa `/get_map`)

**Terminal 1 – jogo:**

```bash
source /opt/ros/jazzy/setup.bash
cd fernandoliveira05-ros2-challenge
source install/setup.bash

ros2 run cg maze
```

**Terminal 2 – solver estático:**

```bash
source /opt/ros/jazzy/setup.bash
cd fernandoliveira05-ros2-challenge
source install/setup.bash

ros2 run cg_solver maze_solver
```

Fluxo:

1. O jogo sobe e oferece o serviço `/get_map`.
2. O nó `maze_solver`:
   - chama `/get_map`;
   - reconstrói o grid;
   - roda BFS do `r` até o `t`;
   - envia a sequência de comandos pelo serviço `/move_command`.

---

### 2. Executar apenas o `mapear` (exploração + caminho ótimo)

**Terminal 1 – jogo:**

```bash
source /opt/ros/jazzy/setup.bash
cd fernandoliveira05-ros2-challenge
source install/setup.bash

ros2 run cg maze
```

**Terminal 2 – mapeador:**

```bash
source /opt/ros/jazzy/setup.bash
cd fernandoliveira05-ros2-challenge
source install/setup.bash

ros2 run cg_solver mapear
```

Fluxo:

1. O jogo sobe e começa a publicar sensores do robô (ex.: `/culling_games/robot_sensors`) e a aceitar comandos em `/move_command`.
2. O nó `mapear`:
   - começa na posição inicial;
   - lê os sensores em cada callback;
   - atualiza o mapa interno (`map_`);
   - explora o ambiente com DFS, usando backtracking;
   - quando a exploração termina e o objetivo foi visto, roda BFS para encontrar o melhor caminho;
   - retorna ao start e executa a rota ótima até o goal.

---

## Dicas de depuração

- Listar serviços e tópicos disponíveis:

  ```bash
  ros2 topic list
  ros2 service list
  ```

- Ver o que o robô “enxerga”:

  ```bash
  ros2 topic echo /culling_games/robot_sensors
  ```

- Conferir logs dos nós:

  - Os nós em C++ usam `RCLCPP_INFO`, `RCLCPP_WARN` e `RCLCPP_ERROR`.  
  - Rode com `--ros-args --log-level debug` se precisar de mais detalhes.

---

## Conclusão

Este repositório reúne:

- um simulador de labirinto em PyGame integrado com ROS 2;
- um solver estático (`maze_solver`), que resolve o labirinto diretamente a partir do mapa completo;
- um solver incremental (`mapear`), que explora, mapeia e depois calcula o caminho ótimo.

Os dois códigos em C++ foram escritos para demonstrar, na prática:

- como modelar um ambiente 2D como grafo,
- como aplicar BFS para encontrar caminhos mínimos em grades com custo unitário,
- e como integrar esses conceitos de grafos com ROS 2 e um simulador de robô.

Sinta‑se à vontade para abrir `src/cg_solver/src/solver.cpp` e `src/cg_solver/src/mapear.cpp` enquanto roda os nós para ver, em tempo real, como cada parte do código se conecta com o comportamento do robô no jogo.
