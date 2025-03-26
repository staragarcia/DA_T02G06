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
