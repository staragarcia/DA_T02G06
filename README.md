# README

## Como utilizar a ferramenta de planeamento de rotas:
### Compilação
1. Ter um compilador C++ instalado (ex.: g++).
2. Compilar o programa com o seguinte comando no terminal: g++ -o route_planner main.cpp
3. Executar o programa: ./route_planner

### Menu de opções
Ao iniciar o programa, será apresentado um menu com as seguintes opções:
1. Independent Route Planning -Calcula a melhor rota e uma rota alternativa entre dois pontos, sem restrições adicionais.

2. Restricted Route Planning - Permite calcular uma rota evitando certos nós ou segmentos, ou incluindo um nó obrigatório.

3. Environmentally-Friendly Route Planning - Calcula uma rota que combina condução e caminhada, respeitando um tempo máximo de caminhada.

4. Run Batch Mode - Processa rotas automaticamente a partir de ficheiros de entrada.

5. Exit - Encerra o programa.

### Notas
- Certificar-se de que os ficheiros de entrada (localizações e distâncias) estão corretamente formatados e disponíveis no diretório do programa.
- Para cada funcionalidade, seguir as instruções no terminal para introduzir os dados necessários (ex.: IDs de origem e destino, nós a evitar, etc.).  

---

## Alterações feitas ao ficheiro graph.h
Para a realização deste projeto, utilizámos o ficheiro graph.h disponibilizado nas aulas práticas de DA, conforme indicado no enunciado. De forma a implementar aquilo que nos foi pedido, tivemos de fazer algumas pequenas alterações e adições a este ficheiro, nomeadamente nas classes Vertex e Edge.  

### Vertex class
No nosso projeto, cada Vertex representa uma localização, com um nome, um id e um código únicos, e informação sobre a existência (ou não) de estacionamento nesse local. Para isso, foi necessário realizar a seguinte alteração:  
- Retirámos o atributo "info" que substituimos por "location", "id", "code" e "parking"  
    **Construtor original**: Vertex(T in);  
    **Construtor modificado**: Vertex(std::string location, T id, std::string code, bool parking);

### Edge class
Na nossa implementação, cada Edge representa a distância entre duas localizações, a qual deve ser medida tanto em tempo de percurso a pé como de carro. Para isso, foi necessário realizar a seguinte alteração:  
- Retirámos o atributo "weight" que substituimos por "drivingTime" e "walkingTime"  
    **Construtor original**: Edge(Vertex<T> *orig, Vertex<T> *dest, double w);  
    **Construtor modificado**: Edge(Vertex<T> *orig, Vertex<T> *dest, int drivingTime, int walkingTime);
  
Em ambas as classes, para cada um dos atributos adicionados, foi também adicionado o respetivo getter.


### Mudanças adicionais:
- Adição de dois cleanup methods:  
   **cleanUpVisitedAndDist:** Responsável por limpar os atributos relacionados com o estado (visitado ou não) e as distâncias de cada vértice;  
   **cleanUpPaths:** Responsável por limpar os caminhos (paths) associados aos vértices;  
  
- Adição de um método à classe Graph:  
   **findVertexById:** Permite procurar um vértice através do seu ID;  
