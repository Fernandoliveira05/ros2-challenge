# PONDERADA SEMANA 06 - Navegação e mapeamento

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
&nbsp;&nbsp;O primeiro algoritmo utiliza o serviço "/get_map" para obter a matriz do labirinto, converte esse mapa em uma representação linear e, a partir disso, interpreta as relações entre os nós para permitir que o robô navegue até o objetivo final de maneira rápida e prática. Convenhamos que esse é mais fácil, porque já recebemos do serviço toda a estrutura do mapa. </br>
&nbsp;&nbsp;O segundo algoritmo é mais elaborado. Antes de buscar o alvo, o robô precisa explorar o mapa por completo: identificar regiões livres, registrar obstáculos, construir um grafo navegável e, somente depois disso, retornar ao ponto de origem. Com o mapa completo reconstruído, o sistema então aplica o algoritmo de Dijkstra para calcular a rota mais eficiente entre a posição inicial e o objetivo. 

