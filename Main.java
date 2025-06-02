import java.util.*;

public class Main {

    private static final String SEPARADOR_LINHA = "\n----------------------------------------------------\n";
    private static final String INDENT = "  ";

    /**
     * Representa uma aresta em um grafo. Pode ser ponderada ou não. Inclui 'origem' e 'destino'. 'id' é usado para Função de Incidência.
     * Implementa Comparable para ordenação por peso (útil em Kruskal, Prim).
     */
    static class Aresta implements Comparable<Aresta> {
        int origem;
        int destino;
        int peso;
        String id;

        // Construtor para arestas ponderadas (comuns em Dijkstra, Prim, Kruskal, Bellman-Ford)
        Aresta(int origem, int destino, int peso) {
            this.origem = origem;
            this.destino = destino;
            this.peso = peso;
        }

        // Construtor para arestas com ID (usado na Função de Incidência)
        Aresta(String id, int origem, int destino) {
            this.id = id;
            this.origem = origem;
            this.destino = destino;
        }

        @Override
        public int compareTo(Aresta outra) {
            // Essencial para algoritmos que ordenam arestas por peso (Kruskal) ou usam PriorityQueue baseada em peso (Prim, Dijkstra).
            return Integer.compare(this.peso, outra.peso);
        }

        @Override
        public String toString() {
            if (id != null) return String.format("Aresta '%s': (%d-%d)", id, origem, destino);
            return String.format("(%d-%d, w:%d)", origem, destino, peso);
        }
    }

    /**
     * Estrutura de dados Union-Find (Conjuntos Disjuntos). Otimizada com "Path Compression" e "Union by Rank". Crucial para a eficiência do algoritmo de Kruskal.
     */
    static class UnionFind {
        private int[] pai;  // pai[i] é o pai do elemento i
        private int[] rank; // rank[i] é uma estimativa da altura da árvore rooted em i (usado em Union by Rank)

        public UnionFind(int n) {
            pai = new int[n];
            rank = new int[n];
            // Inicialmente, cada elemento é seu próprio pai (conjunto unitário) e rank 0.
            for (int i = 0; i < n; i++) {
                pai[i] = i;
                rank[i] = 0;
            }
        }

        /**
         * Encontra o representante (raiz) do conjunto ao qual 'i' pertence.
         * Implementa "Path Compression": faz todos os nós no caminho de 'i' até a raiz apontarem diretamente para a raiz. Isso achata a árvore, acelerando futuras operações find.
         */
        public int find(int i) {
            if (pai[i] == i) {
                return i;
            }
            pai[i] = find(pai[i]); // Recursão com Path Compression
            return pai[i];
        }

        /**
         * Une os conjuntos que contêm 'i' e 'j'. Implementa "Union by Rank": anexa a árvore de menor rank à raiz da árvore de maior rank.
         * Se os ranks são iguais, um é escolhido arbitrariamente e seu rank é incrementado. Retorna true se a união foi realizada (i e j estavam em conjuntos diferentes), false caso contrário.
         */
        public boolean union(int i, int j) {
            int raizI = find(i);
            int raizJ = find(j);

            if (raizI != raizJ) { // Só une se estiverem em conjuntos diferentes
                if (rank[raizI] < rank[raizJ]) {
                    pai[raizI] = raizJ;
                } else if (rank[raizI] > rank[raizJ]) {
                    pai[raizJ] = raizI;
                } else {
                    pai[raizJ] = raizI; // Escolhe um (ex: raizI) como novo pai
                    rank[raizI]++;      // Incrementa o rank
                }
                return true;
            }
            return false;
        }

        public String toString(int numVertices) {
            Map<Integer, List<Integer>> sets = new HashMap<>();
            for(int i=0; i<numVertices; i++){
                sets.computeIfAbsent(find(i), k -> new ArrayList<>()).add(i);
            }
            StringBuilder sb = new StringBuilder();
            for(List<Integer> set : sets.values()){
                sb.append(set.toString()).append(" ");
            }
            return sb.toString().trim();
        }
    }

    static class RepresentacaoGrafo {
        /**
         * Demonstra a representação de grafos usando Lista de Adjacências. Cada vértice tem uma lista de seus vizinhos.
         * Eficiente em espaço para grafos esparsos (poucas arestas). Operações como encontrar vizinhos de um vértice são rápidas (proporcional ao grau do vértice).
         * Verificar se uma aresta (u,v) existe pode ser mais lento (O(grau(u))).
         */
        public static void demonstrarListaAdjacencia(Scanner scanner) {
            System.out.println("\n### Demonstração: Lista de Adjacência ###");
            System.out.println("Representa um grafo simples não direcionado. Para cada vértice v, produz N(v) e o grau de v.\n");

            int n = 4; // Número de vértices
            List<int[]> arestasInput = Arrays.asList( // Arestas do grafo de exemplo
                    new int[]{0, 1}, new int[]{0, 2}, new int[]{1, 2}, new int[]{2, 3}
            );
            System.out.println(INDENT + "Grafo de Exemplo:");
            System.out.println(INDENT + "Número de Vértices: " + n);
            System.out.println(INDENT + "Arestas (u,v):");
            for (int[] aresta : arestasInput) System.out.println(INDENT + INDENT + Arrays.toString(aresta));
            System.out.println();

            // Cria um array de LinkedLists. adj[i] armazena os vizinhos do vértice i.
            @SuppressWarnings("unchecked")
            LinkedList<Integer>[] adj = new LinkedList[n];
            for (int i = 0; i < n; i++) {
                adj[i] = new LinkedList<>();
            }

            // Adiciona as arestas. Para grafos não direcionados, adiciona u em adj[v] e v em adj[u].
            for (int[] aresta : arestasInput) {
                adj[aresta[0]].add(aresta[1]);
                adj[aresta[1]].add(aresta[0]);
            }

            System.out.println(INDENT + "Saída Gerada (N(v) e grau(v)):");
            for (int v = 0; v < n; v++) {
                Collections.sort(adj[v]); // Opcional: ordena para saída consistente.
                // N(v) é a lista de adjacência de v.
                // grau(v) é o tamanho da lista de adjacência de v.
                System.out.printf(INDENT + "N(%d)=%s, grau=%d\n", v, adj[v].toString(), adj[v].size());
            }
        }

        /**
         * Demonstra a representação de grafos usando Função de Incidência.
         * Associa cada aresta (identificada por uma String) ao conjunto de dois vértices que ela conecta. Útil quando as arestas têm identidades próprias ou atributos.
         */
        public static void demonstrarFuncaoIncidencia(Scanner scanner) {
            System.out.println("\n### Demonstração: Função de Incidência ###");
            System.out.println("Associa cada aresta (com ID textual) ao conjunto de vértices que ela conecta (phi(e)={u,v}).\n");

            List<Aresta> arestasInput = Arrays.asList(
                    new Aresta("e1", 0, 1), new Aresta("e2", 0, 2), new Aresta("e3", 1, 3)
            );
            System.out.println(INDENT + "Grafo de Exemplo:");
            System.out.println(INDENT + "Arestas (id, v1, v2):");
            for (Aresta a : arestasInput) System.out.println(INDENT + INDENT + a.id + " " + a.origem + " " + a.destino);
            System.out.println();

            // Usa um Map onde a chave é o ID da aresta e o valor é um Set dos dois vértices.
            Map<String, Set<Integer>> funcaoIncidencia = new HashMap<>();
            for (Aresta a : arestasInput) {
                Set<Integer> verticesConectados = new HashSet<>();
                verticesConectados.add(a.origem);
                verticesConectados.add(a.destino);
                funcaoIncidencia.put(a.id, verticesConectados);
            }

            System.out.println(INDENT + "Saída Gerada (Função de Incidência phi(e)={u,v}):");
            for (Map.Entry<String, Set<Integer>> entry : funcaoIncidencia.entrySet()) {
                List<Integer> verticesOrdenados = new ArrayList<>(entry.getValue());
                Collections.sort(verticesOrdenados);
                System.out.printf(INDENT + "phi(%s)=%s\n", entry.getKey(), verticesOrdenados.toString());
            }
        }
    }

    /**
     * Demonstra algoritmos de busca em grafos: DFS e BFS.
     */
    static class BuscasGrafo {
        /**
         * Demonstra a Busca em Profundidade (DFS - Depth-First Search).
         * Explora o grafo "indo o mais fundo possível" por um caminho antes de retroceder (backtracking). Usa uma Pilha (Stack) para controlar os vértices a serem visitados (explícita ou implícita via recursão).
         * Complexidade: O(V+E) onde V é o número de vértices e E o número de arestas. Aplicações: detecção de ciclos, ordenação topológica, componentes conectados.
         */
        public static void demonstrarDFS(Scanner scanner) {
            System.out.println("\n### Demonstração: Busca em Profundidade (DFS) ###");
            System.out.println("Realiza DFS a partir do vértice 0 e imprime a ordem dos vértices visitados.\n");

            int n = 6;
            List<int[]> arestasInput = Arrays.asList(
                    new int[]{0,1}, new int[]{0,2}, new int[]{1,3}, new int[]{2,4}, new int[]{4,5}
            );
            System.out.println(INDENT + "Grafo de Exemplo:");
            System.out.println(INDENT + "Vértices: " + n);
            System.out.println(INDENT + "Arestas: " + arestasInput.stream().map(Arrays::toString).reduce((s1,s2) -> s1 + ", " + s2).orElse(""));
            System.out.println();


            @SuppressWarnings("unchecked")
            List<Integer>[] adj = new List[n];
            for (int i = 0; i < n; i++) adj[i] = new ArrayList<>();
            for (int[] aresta : arestasInput) {
                adj[aresta[0]].add(aresta[1]);
                adj[aresta[1]].add(aresta[0]);
            }
            for (int i = 0; i < n; i++) Collections.sort(adj[i]); // Garante ordem de visita determinística

            boolean[] visitado = new boolean[n]; // Marca vértices já visitados
            List<Integer> ordemVisita = new ArrayList<>(); // Armazena a ordem de visita
            Stack<Integer> pilha = new Stack<>(); // Pilha para a implementação iterativa do DFS

            System.out.println(INDENT + "Execução Passo a Passo (Origem Vértice 0):");
            System.out.println(INDENT + String.format("%-20s | %-20s | %s", "Pilha (Topo->Base)", "Visitados", "Ordem de Visita"));
            System.out.println(INDENT + "------------------------------------------------------------------");

            pilha.push(0); // Começa pelo vértice 0

            int passo = 0;
            while (!pilha.isEmpty()) {
                int u = pilha.pop(); // Pega o próximo vértice da pilha
                if (!visitado[u]) {
                    visitado[u] = true; // Marca como visitado
                    ordemVisita.add(u);
                    System.out.printf(INDENT + "%-20s | %-20s | %s (Processando %d)\n",
                            formatStack(pilha), formatArray(visitado), ordemVisita, u);

                    // Adiciona vizinhos não visitados à pilha. A ordem de adição à pilha influencia a ordem de exploração.
                    // Adicionando em ordem reversa (após ordenar adjacências) faz com que vizinhos de menor índice sejam processados primeiro (se desejado).
                    List<Integer> vizinhos = new ArrayList<>(adj[u]);
                    Collections.reverse(vizinhos);
                    for (int v : vizinhos) {
                        if (!visitado[v]) {
                            pilha.push(v);
                        }
                    }
                }
                // Imprime o estado da pilha mesmo se o vértice já foi visitado (quando pop acontece)
                if(pilha.isEmpty() || visitado[u]) { // Evita duplicar a linha de "Processando"
                    System.out.printf(INDENT + "%-20s | %-20s | %s\n",
                            formatStack(pilha), formatArray(visitado), ordemVisita);
                }


                passo++;
                if (passo > n*n && !pilha.isEmpty()) {
                    System.out.println(INDENT + "Loop de DFS muito longo, interrompendo.");
                    break;
                }
            }

            System.out.println(INDENT + "------------------------------------------------------------------");
            System.out.println(INDENT + "DFS final (ordem de visita) a partir do vértice 0:");
            System.out.print(INDENT + INDENT);
            for (int i = 0; i < ordemVisita.size(); i++) {
                System.out.print(ordemVisita.get(i) + (i == ordemVisita.size() - 1 ? "" : " "));
            }
            System.out.println("\n");
        }

        /**
         * Demonstra a Busca em Largura (BFS - Breadth-First Search).
         * Explora o grafo "camada por camada", visitando todos os vizinhos de um nó antes de prosseguir. Usa uma Fila (Queue) para controlar os vértices a serem visitados.
         * Complexidade: O(V+E). Aplicações: encontrar o caminho mais curto em grafos não ponderados, componentes conectados.
         */
        public static void demonstrarBFS(Scanner scanner) {
            System.out.println("\n### Demonstração: Busca em Largura (BFS) ###");
            System.out.println("Encontra a distância mínima (número de arestas) entre dois vértices.\n");

            int n = 6;
            List<int[]> arestasInput = Arrays.asList(
                    new int[]{0,1}, new int[]{0,3}, new int[]{1,5}, new int[]{2,5}, new int[]{3,4}, new int[]{4,5}
            );
            int origem = 0;
            int destino = 5;
            System.out.println(INDENT + "Grafo de Exemplo:");
            System.out.println(INDENT + "Vértices: " + n);
            System.out.println(INDENT + "Arestas: " + arestasInput.stream().map(Arrays::toString).reduce((s1,s2) -> s1 + ", " + s2).orElse(""));
            System.out.println(INDENT + "Origem: " + origem + ", Destino: " + destino);
            System.out.println();


            @SuppressWarnings("unchecked")
            List<Integer>[] adj = new List[n];
            for (int i = 0; i < n; i++) adj[i] = new ArrayList<>();
            for (int[] aresta : arestasInput) {
                adj[aresta[0]].add(aresta[1]);
                adj[aresta[1]].add(aresta[0]);
            }
            for (int i = 0; i < n; i++) Collections.sort(adj[i]);

            Queue<Integer> fila = new LinkedList<>(); // Fila para o BFS
            boolean[] visitado = new boolean[n];     // Marca visitados
            int[] distancia = new int[n];          // Armazena distância da origem
            int[] predecessor = new int[n];        // Armazena predecessor no caminho
            Arrays.fill(distancia, -1);      // Inicializa distâncias como -1 (não alcançável)
            Arrays.fill(predecessor, -1);    // Inicializa predecessores

            fila.add(origem);
            visitado[origem] = true;
            distancia[origem] = 0;

            System.out.println(INDENT + "Execução Passo a Passo (Origem " + origem + ", Destino " + destino + "):");
            System.out.println(INDENT + String.format("%-20s | %-20s | %-20s | %s", "Fila (Frente->Fim)", "Visitados", "Distâncias", "Processando Vértice"));
            System.out.println(INDENT + "-------------------------------------------------------------------------------------");

            int u = -1;
            while(!fila.isEmpty()) {
                u = fila.poll(); // Pega o próximo da fila
                System.out.printf(INDENT + "%-20s | %-20s | %-20s | %s\n",
                        formatQueue(fila), formatArray(visitado), formatDistPred(distancia, null, n), u);

                if (u == destino) break; // Se chegou ao destino, para.

                // Para cada vizinho v de u
                for (int v : adj[u]) {
                    if (!visitado[v]) { // Se v não foi visitado
                        visitado[v] = true;
                        distancia[v] = distancia[u] + 1; // Distância de v é dist(u) + 1
                        predecessor[v] = u;              // u é predecessor de v
                        fila.add(v);                     // Adiciona v à fila
                    }
                }
            }
            System.out.printf(INDENT + "%-20s | %-20s | %-20s | %s (Fila Vazia ou Destino Encontrado)\n",
                    formatQueue(fila), formatArray(visitado), formatDistPred(distancia, null, n), "-");
            System.out.println(INDENT + "-------------------------------------------------------------------------------------");


            if (distancia[destino] != -1) {
                System.out.println(INDENT + "Distância mínima de " + origem + " até " + destino + " é " + distancia[destino]);
                // Reconstruir caminho usando o array de predecessores
                List<Integer> caminho = new ArrayList<>();
                int curr = destino;
                while(curr != -1) {
                    caminho.add(curr);
                    curr = predecessor[curr];
                }
                Collections.reverse(caminho);
                System.out.println(INDENT + "Caminho: " + caminho.toString().replace("[","").replace("]","").replace(", "," -> "));

            } else {
                System.out.println(INDENT + "Não há caminho de " + origem + " até " + destino);
            }
        }
    }

    /**
     * Demonstra algoritmos para encontrar caminhos de custo mínimo: Dijkstra e Bellman-Ford.
     */
    static class CaminhosMinimos {
        /**
         * Demonstra o Algoritmo de Dijkstra.
         * Encontra os caminhos de menor custo de um vértice de origem para todos os outros vértices em um grafo ponderado com pesos de aresta não-negativos.
         * Guloso: sempre escolhe a aresta de menor peso para um vértice não visitado.
         * Usa uma Fila de Prioridade (Min-Priority Queue) para selecionar eficientemente o próximo vértice a visitar. Complexidade: O((V+E)logV) com Fila de Prioridade implementada com heap binário, ou O(V^2) com array simples.
         */
        public static void demonstrarDijkstra(Scanner scanner) {
            System.out.println("\n### Demonstração: Algoritmo de Dijkstra ###");
            System.out.println("Calcula os caminhos de menor custo de uma origem para todos os outros vértices (pesos de aresta não-negativos).\n");

            int n = 5;
            List<Aresta> arestasInput = Arrays.asList( /* ... definição das arestas ... */
                    new Aresta(0, 1, 10), new Aresta(0, 3, 5),
                    new Aresta(1, 2, 1), new Aresta(1, 3, 2),
                    new Aresta(2, 4, 4),
                    new Aresta(3, 1, 3), new Aresta(3, 2, 9), new Aresta(3, 4, 2),
                    new Aresta(4, 0, 7), new Aresta(4, 2, 6)
            );
            int origem = 0;
            System.out.println(INDENT + "Grafo de Exemplo Ponderado:");
            System.out.println(INDENT + "Vértices: " + n + ", Origem: " + origem);
            System.out.println(INDENT + "Arestas (origem, destino, peso):");
            for(Aresta a : arestasInput) System.out.println(INDENT + INDENT + a);
            System.out.println();


            @SuppressWarnings("unchecked")
            List<Aresta>[] adj = new List[n]; // Lista de adjacência (com pesos)
            for(int i=0; i<n; i++) adj[i] = new ArrayList<>();
            for(Aresta a : arestasInput) adj[a.origem].add(a);

            int[] dist = new int[n];          // dist[i] armazena o custo do caminho mínimo da origem até i
            int[] pred = new int[n];          // pred[i] armazena o predecessor de i no caminho mínimo
            boolean[] visitado = new boolean[n]; // Marca se o caminho mínimo para o vértice já foi finalizado
            Arrays.fill(dist, Integer.MAX_VALUE); // Inicializa distâncias como infinito
            Arrays.fill(pred, -1);                // Inicializa predecessores como -1

            dist[origem] = 0; // Distância da origem para ela mesma é 0
            // Fila de Prioridade armazena Aresta(origem_ficticia, vertice_a_visitar, custo_acumulado_para_vertice_a_visitar)
            // Ordenada pelo custo_acumulado.
            PriorityQueue<Aresta> pq = new PriorityQueue<>(Comparator.comparingInt(a -> a.peso));
            pq.add(new Aresta(-1, origem, 0));

            System.out.println(INDENT + "Execução Passo a Passo (Origem " + origem + "):");
            System.out.println(INDENT + String.format("%-15s | %-25s | %-20s | %-15s | %s", "Processando", "Distâncias", "Predecessores", "Visitados", "PQ (vtx, custo)"));
            System.out.println(INDENT + "----------------------------------------------------------------------------------------------------------");
            int passo = 0;

            while(!pq.isEmpty()) {
                // Mostra o estado ANTES de retirar da PQ e processar
                System.out.printf(INDENT + "%-15s | %-25s | %-20s | %-15s | %s\n",
                        "-", formatDistPred(dist,null,n), formatDistPred(pred,null,n), formatArray(visitado), formatPQ(pq));

                Aresta arestaMinima = pq.poll(); // Extrai o vértice 'u' com menor dist[u] da PQ
                int u = arestaMinima.destino;

                // Se u já foi visitado (caminho finalizado), ignora.
                // Isso ocorre se adicionamos múltiplas entradas para o mesmo vértice na PQ com custos diferentes (uma otimização comum é usar DecreaseKey).
                if (visitado[u]) continue;
                visitado[u] = true; // Marca u como visitado (caminho finalizado)

                System.out.printf(INDENT + "%-15s | %-25s | %-20s | %-15s | %s (Processando %d)\n",
                        "Vértice " + u, formatDistPred(dist,null,n), formatDistPred(pred,null,n), formatArray(visitado), formatPQ(pq), u);

                // Para cada vizinho 'v' de 'u'
                for(Aresta vizinha : adj[u]) {
                    int v = vizinha.destino;
                    int pesoUV = vizinha.peso;
                    // Se 'v' não foi visitado e um caminho mais curto para 'v' via 'u' foi encontrado
                    if (!visitado[v] && dist[u] != Integer.MAX_VALUE && dist[u] + pesoUV < dist[v]) {
                        System.out.printf(INDENT + INDENT + "Relaxando (%d,%d) peso %d: dist[%d] de %s para %d.\n", u,v,pesoUV, v, dist[v]==Integer.MAX_VALUE?"Inf":dist[v], dist[u]+pesoUV);
                        dist[v] = dist[u] + pesoUV; // Atualiza dist[v]
                        pred[v] = u;                // Define u como predecessor de v
                        pq.add(new Aresta(u, v, dist[v])); // Adiciona (ou atualiza) v na PQ com a nova distância
                    }
                }
                passo++;
                if(passo > n * n * adj.length ) { System.out.println(INDENT + "Loop de Dijkstra muito longo. (safety break)"); break; }
            }
            System.out.printf(INDENT + "%-15s | %-25s | %-20s | %-15s | %s (Fim)\n",
                    "-", formatDistPred(dist,null,n), formatDistPred(pred,null,n), formatArray(visitado), formatPQ(pq));
            System.out.println(INDENT + "----------------------------------------------------------------------------------------------------------");

            System.out.println(INDENT + "Resultados Finais (Caminhos Mínimos a partir de " + origem + "):");
            for (int i = 0; i < n; i++) {
                if (dist[i] == Integer.MAX_VALUE) {
                    System.out.printf(INDENT + "Para %d: Sem caminho (custo Inf)\n", i);
                } else {
                    List<Integer> caminho = new ArrayList<>();
                    List<Integer> pesosCaminho = new ArrayList<>();
                    int curr = i;
                    while (curr != -1 && pred[curr] != -1) {
                        caminho.add(curr);
                        int p = pred[curr];
                        for (Aresta a : adj[p]) { // Encontra o peso da aresta (p -> curr)
                            if (a.destino == curr) {
                                pesosCaminho.add(a.peso);
                                break;
                            }
                        }
                        curr = p;
                    }
                    if(i == origem || pred[i] != -1 || dist[i] == 0) caminho.add(origem); // Adiciona origem se não foi adicionada
                    else if(caminho.isEmpty() && i!=origem) { /* Não deveria acontecer se dist[i] != MAX_VALUE */ }


                    Collections.reverse(caminho);
                    Collections.reverse(pesosCaminho);

                    StringBuilder sbPath = new StringBuilder();
                    if (!caminho.isEmpty()) sbPath.append(caminho.get(0));

                    for(int k=1; k < caminho.size(); k++){
                        if (k-1 < pesosCaminho.size()) { // Garante que há peso para a aresta
                            sbPath.append(" ->(").append(pesosCaminho.get(k-1)).append(") ").append(caminho.get(k));
                        } else { // Se não encontrar peso (ex: origem para si mesma), só o vértice
                            sbPath.append(" -> ").append(caminho.get(k));
                        }
                    }
                    if (caminho.isEmpty() && i == origem) sbPath.append(origem); // Caso especial: caminho para a própria origem

                    System.out.printf(INDENT + "Para %d: %s (custo %d)\n", i, sbPath.toString(), dist[i]);
                }
            }
        }

        /**
         * Demonstra o Algoritmo de Bellman-Ford.
         * Encontra os caminhos de menor custo de uma origem para todos os outros vértices. Funciona com pesos de aresta negativos. Pode detectar ciclos de peso negativo acessíveis a partir da origem.
         * Lógica: Relaxa todas as arestas |V|-1 vezes. Se na |V|-ésima vez ainda houver relaxamento, há um ciclo negativo. Complexidade: O(V*E). Mais lento que Dijkstra, mas mais geral.
         */
        public static void demonstrarBellmanFord(Scanner scanner) {
            System.out.println("\n### Demonstração: Algoritmo de Bellman-Ford ###");
            System.out.println("Calcula caminhos de menor custo e detecta ciclos negativos.\n");

            // Grafo do PDF 07, Ex1. Mapeando S=0, t=1, x=2, y=3, z=4
            int n = 5;
            int origem = 0; // S
            List<Aresta> arestas = Arrays.asList(
                    new Aresta(0, 2, 6),  new Aresta(0, 4, 7),  new Aresta(1, 2, 8),
                    new Aresta(1, 3, 5),  new Aresta(2, 3, -2), new Aresta(3, 0, -3),
                    new Aresta(3, 4, 8),  new Aresta(4, 1, -4), new Aresta(4, 0, 2)
                    // Para testar ciclo negativo, adicionar: new Aresta(2,0,-7) -> S-x-S = 6-7=-1
            );
            Map<Integer, String> nomeVertice = new HashMap<>();
            nomeVertice.put(0,"S"); nomeVertice.put(1,"t"); nomeVertice.put(2,"x"); nomeVertice.put(3,"y"); nomeVertice.put(4,"z");
            System.out.println(INDENT + "Grafo de Exemplo Ponderado (com possíveis pesos negativos):");
            System.out.println(INDENT + "Vértices: " + n + " ("+ nomeVertice.values() +"), Origem: " + nomeVertice.get(origem));
            System.out.println(INDENT + "Arestas (origem, destino, peso):");
            for(Aresta a : arestas) System.out.printf(INDENT + INDENT + "(%s -> %s, w:%d)\n", nomeVertice.get(a.origem), nomeVertice.get(a.destino), a.peso);
            System.out.println();


            int[] dist = new int[n];
            int[] pred = new int[n];
            Arrays.fill(dist, Integer.MAX_VALUE);
            Arrays.fill(pred, -1);
            dist[origem] = 0;

            System.out.println(INDENT + "Execução Passo a Passo (Origem " + nomeVertice.get(origem) + "):");
            System.out.println(INDENT + String.format("%-10s | %-25s | %s", "Iteração", "Distâncias (S,t,x,y,z)", "Predecessores (S,t,x,y,z)"));
            System.out.println(INDENT + "-----------------------------------------------------------------------");
            System.out.printf(INDENT + "%-10s | %-25s | %s\n", "Inicial", formatDistPred(dist, nomeVertice, n), formatDistPred(pred, nomeVertice, n));

            // Fase de Relaxamento: Repete |V|-1 vezes
            for (int i = 1; i < n; i++) {
                boolean mudouNestaIteracao = false;
                System.out.printf(INDENT + "Iteração %d:\n", i);
                // Para cada aresta (u,v) no grafo
                for (Aresta aresta : arestas) {
                    // Se dist[u] não é infinito E dist[u] + peso(u,v) < dist[v]
                    if (dist[aresta.origem] != Integer.MAX_VALUE &&
                            dist[aresta.origem] + aresta.peso < dist[aresta.destino]) {
                        System.out.printf(INDENT + INDENT + "Relaxando (%s->%s, %d): dist[%s] de %s para %d\n",
                                nomeVertice.get(aresta.origem), nomeVertice.get(aresta.destino), aresta.peso,
                                nomeVertice.get(aresta.destino), dist[aresta.destino]==Integer.MAX_VALUE?"Inf":String.valueOf(dist[aresta.destino]),
                                dist[aresta.origem] + aresta.peso);
                        dist[aresta.destino] = dist[aresta.origem] + aresta.peso; // Relaxa a aresta
                        pred[aresta.destino] = aresta.origem;
                        mudouNestaIteracao = true;
                    }
                }
                System.out.printf(INDENT + "%-10s | %-25s | %s\n", "Fim It."+i, formatDistPred(dist, nomeVertice, n), formatDistPred(pred, nomeVertice, n));
                if (!mudouNestaIteracao && i < n-1) {
                    System.out.println(INDENT + "Nenhuma distância alterada na iteração " + i + ". Convergiu antecipadamente.");
                    break; // Otimização: se não houve mudança, não haverá nas próximas.
                }
            }

            System.out.println(INDENT + "-----------------------------------------------------------------------");
            // Fase de Detecção de Ciclo Negativo: |V|-ésima iteração
            boolean cicloNegativoEncontrado = false;
            List<Integer> ciclo = new ArrayList<>();
            int verticeNoCiclo = -1; // Vértice que será atualizado, indicando um ciclo.

            System.out.println(INDENT + "Verificando ciclos negativos (Iteração |V|):");
            for (Aresta aresta : arestas) {
                if (dist[aresta.origem] != Integer.MAX_VALUE &&
                        dist[aresta.origem] + aresta.peso < dist[aresta.destino]) {
                    System.out.printf(INDENT + INDENT + "Ciclo negativo detectado! Aresta (%s->%s, %d) ainda pode ser relaxada.\n",
                            nomeVertice.get(aresta.origem), nomeVertice.get(aresta.destino), aresta.peso);
                    cicloNegativoEncontrado = true;
                    // Para exibir o ciclo, marcamos um vértice que foi atualizado e seguimos predecessores.pred[aresta.destino] será atualizado para aresta.origem se o relaxamento fosse feito.
                    // O vértice 'aresta.destino' é parte do ciclo ou alcançável por ele.
                    verticeNoCiclo = aresta.destino;
                    // Forçar a atualização para rastrear o ciclo corretamente com os 'pred' do ciclo
                    // dist[aresta.destino] = dist[aresta.origem] + aresta.peso; // Não é estritamente necessário para detecção, mas ajuda a rastrear.
                    // pred[aresta.destino] = aresta.origem;
                    break;
                }
            }

            if (cicloNegativoEncontrado) {
                System.out.print(INDENT + "Ciclo Negativo Identificado (um possível ciclo): ");
                // Para encontrar o ciclo: a partir do 'verticeNoCiclo', siga os predecessores 'n' vezes
                // para garantir que você esteja "dentro" do ciclo.
                int vAtual = verticeNoCiclo;
                for(int k=0; k<n; ++k) {
                    if(pred[vAtual] == -1) { // Se não há predecessor, não podemos rastrear.
                        System.out.println("Não foi possível rastrear o ciclo completo (predecessor ausente).");
                        ciclo.clear(); // Limpa para não imprimir ciclo parcial errado.
                        break;
                    }
                    vAtual = pred[vAtual];
                }

                if (!ciclo.isEmpty() || pred[vAtual]!=-1) { // Se ainda temos um ponto de partida
                    int vCiclo = vAtual; // Este vértice está garantidamente no ciclo ou leva a ele.
                    boolean[] visitadosNoCiclo = new boolean[n];
                    int count = 0;
                    while(!visitadosNoCiclo[vCiclo] && count < n*2 && pred[vCiclo]!=-1) { // Proteção
                        visitadosNoCiclo[vCiclo] = true;
                        ciclo.add(vCiclo);
                        vCiclo = pred[vCiclo];
                        count++;
                    }
                    if(pred[vCiclo]!=-1) ciclo.add(vCiclo); // Adiciona o vértice que fecha o ciclo
                    Collections.reverse(ciclo);

                    // Remove prefixo até a primeira repetição para mostrar apenas o ciclo
                    if(ciclo.size() > 1 && ciclo.get(0).equals(ciclo.get(ciclo.size()-1))) {
                        // Ciclo bem formado
                    } else if (!ciclo.isEmpty()){
                        // Tenta refinar: encontra o primeiro repetido
                        Map<Integer, Integer> firstOccurrence = new HashMap<>();
                        int startCycleIdx = -1;
                        for(int k=0; k<ciclo.size(); ++k){
                            if(firstOccurrence.containsKey(ciclo.get(k))){
                                startCycleIdx = firstOccurrence.get(ciclo.get(k));
                                ciclo = ciclo.subList(startCycleIdx, k+1);
                                break;
                            }
                            firstOccurrence.put(ciclo.get(k), k);
                        }
                    }
                }


                if (ciclo.isEmpty()) {
                    System.out.println(INDENT + "Ciclo negativo existe, mas o rastreamento específico não foi simples.");
                } else {
                    for(int i=0; i<ciclo.size(); ++i) {
                        System.out.print(nomeVertice.get(ciclo.get(i)) + (i == ciclo.size()-1 ? "" : " -> "));
                    }
                    System.out.println();
                }
            } else {
                System.out.println(INDENT + "Nenhum ciclo negativo detectado a partir da origem.");
                System.out.println(INDENT + "\nResultados Finais (Caminhos Mínimos a partir de " + nomeVertice.get(origem) + "):");
                for (int i = 0; i < n; i++) {
                    if (dist[i] == Integer.MAX_VALUE) {
                        System.out.printf(INDENT + "Para %s: Sem caminho (custo Inf)\n", nomeVertice.get(i));
                    } else {
                        List<Integer> caminho = new ArrayList<>();
                        int curr = i;
                        int safety = 0;
                        while(curr != -1 && safety < n * 2) {
                            caminho.add(curr);
                            if (curr == origem) break;
                            curr = pred[curr];
                            safety++;
                        }
                        Collections.reverse(caminho);
                        StringBuilder sbPath = new StringBuilder();
                        for(int k=0; k<caminho.size();++k) sbPath.append(nomeVertice.get(caminho.get(k))).append(k==caminho.size()-1?"":" -> ");
                        System.out.printf(INDENT + "Para %s: %s (custo %d)\n", nomeVertice.get(i), sbPath.toString(), dist[i]);
                    }
                }
            }
        }
    }

    /**
     * Demonstra algoritmos para encontrar a Árvore Geradora Mínima (MST): Kruskal e Prim.
     */
    static class ArvoreGeradoraMinima {
        /**
         * Demonstra o Algoritmo de Kruskal. Encontra uma MST para um grafo conectado, não direcionado e ponderado.
         * Guloso: adiciona arestas em ordem crescente de peso, desde que não formem ciclo.
         * Usa a estrutura Union-Find para verificar eficientemente se adicionar uma aresta forma um ciclo.
         * Complexidade: O(E log E) devido à ordenação das arestas, ou O(E log V) se E é próximo de V^2. As operações de Union-Find são quase O(1) em média com as otimizações.
         */
        public static void demonstrarKruskal(Scanner scanner) {
            System.out.println("\n### Demonstração: Algoritmo de Kruskal ###");
            System.out.println("Encontra uma Árvore Geradora Mínima (MST) usando Union-Find com otimizações.\n");

            int n = 4;
            List<Aresta> arestasInput = new ArrayList<>(Arrays.asList(
                    new Aresta(0, 1, 10), new Aresta(0, 2, 6),
                    new Aresta(0, 3, 5), new Aresta(1, 3, 15),
                    new Aresta(2, 3, 4)
            ));
            System.out.println(INDENT + "Grafo de Exemplo Ponderado:");
            System.out.println(INDENT + "Vértices: " + n);
            System.out.println(INDENT + "Arestas (origem, destino, peso):");
            for(Aresta a : arestasInput) System.out.println(INDENT + INDENT + a);
            System.out.println();


            Collections.sort(arestasInput); // Passo 1: Ordenar todas as arestas por peso crescente.

            UnionFind uf = new UnionFind(n); // Estrutura para detectar ciclos
            List<Aresta> mst = new ArrayList<>(); // Armazena as arestas da MST
            int custoTotal = 0;

            System.out.println(INDENT + "Execução Passo a Passo:");
            System.out.println(INDENT + "Arestas Ordenadas: " + arestasInput);
            System.out.println(INDENT + String.format("%-20s | %-20s | %-25s | %-20s | %s", "Considerando Aresta", "Ação", "Conjuntos (Union-Find)", "MST Parcial", "Custo Parcial"));
            System.out.println(INDENT + "-------------------------------------------------------------------------------------------------------------------");
            System.out.printf(INDENT + "%-20s | %-20s | %-25s | %-20s | %s\n", "-", "-", uf.toString(n), formatArestaList(mst), custoTotal);

            // Para cada aresta, em ordem crescente de peso
            for (Aresta aresta : arestasInput) {
                String acao;
                // Se os vértices da aresta não estão no mesmo conjunto (find(u) != find(v))
                // então adicionar a aresta não forma ciclo.
                if (uf.union(aresta.origem, aresta.destino)) {
                    mst.add(aresta); // Adiciona à MST
                    custoTotal += aresta.peso;
                    acao = "Adicionada à MST";
                } else {
                    acao = "Descartada (forma ciclo)";
                }
                System.out.printf(INDENT + "%-20s | %-20s | %-25s | %-20s | %s\n",
                        aresta.toString(), acao, uf.toString(n), formatArestaList(mst), custoTotal);
                if (mst.size() == n - 1) break; // Otimização: MST tem |V|-1 arestas.
            }
            System.out.println(INDENT + "-------------------------------------------------------------------------------------------------------------------");

            System.out.println(INDENT + "Resultado Final:");
            System.out.println(INDENT + "MST: " + formatArestaList(mst));
            System.out.println(INDENT + "Custo Total da MST: " + custoTotal);
        }

        /**
         * Demonstra o Algoritmo de Prim. Encontra uma MST para um grafo conectado, não direcionado e ponderado.
         * Guloso: constrói a MST iterativamente, começando de um vértice arbitrário e adicionando a aresta de menor peso que conecta um vértice na MST a um vértice fora dela.
         * Usa uma Fila de Prioridade (Min-Priority Queue) para encontrar eficientemente essa aresta. Complexidade: O((V+E)logV) ou O(E logV) com Fila de Prioridade implementada com heap binário.
         */
        public static void demonstrarPrim(Scanner scanner) {
            System.out.println("\n### Demonstração: Algoritmo de Prim ###");
            System.out.println("Encontra uma Árvore Geradora Mínima (MST) começando de um vértice e expandindo.\n");

            int n = 4;
            List<Aresta> arestasInput = Arrays.asList( /* ... mesmas arestas de Kruskal ... */
                    new Aresta(0, 1, 10), new Aresta(0, 2, 6),
                    new Aresta(0, 3, 5), new Aresta(1, 3, 15),
                    new Aresta(2, 3, 4)
            );
            int inicio = 0;
            System.out.println(INDENT + "Grafo de Exemplo Ponderado:");
            System.out.println(INDENT + "Vértices: " + n + ", Vértice inicial: " + inicio);
            System.out.println(INDENT + "Arestas (origem, destino, peso):");
            for(Aresta a : arestasInput) System.out.println(INDENT + INDENT + a);
            System.out.println();


            @SuppressWarnings("unchecked")
            List<Aresta>[] adj = new List[n];
            for(int i=0; i<n; i++) adj[i] = new ArrayList<>();
            for(Aresta a : arestasInput) { // Grafo precisa ser representado como não direcionado para Prim
                adj[a.origem].add(new Aresta(a.origem, a.destino, a.peso));
                adj[a.destino].add(new Aresta(a.destino, a.origem, a.peso));
            }

            boolean[] naMST = new boolean[n];         // Marca se o vértice já está na MST
            int[] custoParaConectar = new int[n];   // Custo mínimo para conectar o vértice à MST (key[] do Cormen)
            int[] paiNaMST = new int[n];            // Predecessor do vértice na MST (pred[] do Cormen)
            Arrays.fill(custoParaConectar, Integer.MAX_VALUE);
            Arrays.fill(paiNaMST, -1);

            // PQ armazena Aresta(origem_na_mst, vertice_fora_mst, custo_aresta)
            // Ordenada pelo custo_aresta.
            PriorityQueue<Aresta> pq = new PriorityQueue<>(Comparator.comparingInt(a -> a.peso));

            custoParaConectar[inicio] = 0; // Custo para conectar o vértice inicial é 0
            pq.add(new Aresta(-1, inicio, 0)); // Adiciona à PQ (origem -1 é fictícia)

            List<Aresta> mst = new ArrayList<>();
            int custoTotal = 0;

            System.out.println(INDENT + "Execução Passo a Passo (Início Vértice " + inicio + "):");
            System.out.println(INDENT + String.format("%-15s | %-20s | %-25s | %-20s | %s", "Vértice Add", "CustoConectar(Key)", "PaiMST(Pred)", "PQ(vtx,custo)", "MST Parcial"));
            System.out.println(INDENT + "--------------------------------------------------------------------------------------------------------------------");
            System.out.printf(INDENT + "%-15s | %-20s | %-25s | %-20s | %s\n", "-", formatDistPred(custoParaConectar,null,n), formatDistPred(paiNaMST,null,n), formatPQ(pq), "[]");

            while(!pq.isEmpty() && mst.size() < n-1) { // Continua enquanto PQ não vazia e MST não completa
                Aresta arestaMin = pq.poll(); // Extrai a aresta de menor custo que conecta à MST
                int u = arestaMin.destino;    // Vértice a ser adicionado à MST

                if (naMST[u]) continue; // Se já está na MST, ignora

                naMST[u] = true; // Adiciona u à MST
                // Se não for o vértice inicial (que tem origem fictícia -1)
                if (arestaMin.origem != -1) {
                    mst.add(new Aresta(arestaMin.origem, u, arestaMin.peso));
                    custoTotal += arestaMin.peso;
                }

                System.out.printf(INDENT + "%-15s | %-20s | %-25s | %-20s | %s (Add %d)\n",
                        u, formatDistPred(custoParaConectar,null,n), formatDistPred(paiNaMST,null,n), formatPQ(pq), formatArestaList(mst), u);

                // Para cada vizinho 'v' de 'u'
                for(Aresta vizinha : adj[u]) {
                    int v = vizinha.destino;
                    int pesoUV = vizinha.peso;
                    // Se 'v' não está na MST e o peso da aresta (u,v) é menor que o custo atual para conectar 'v'
                    if (!naMST[v] && pesoUV < custoParaConectar[v]) {
                        System.out.printf(INDENT + INDENT + "Atualizando custo para %d via (%d,%d) de %s para %d.\n", v, u, v, custoParaConectar[v]==Integer.MAX_VALUE?"Inf":String.valueOf(custoParaConectar[v]), pesoUV);
                        custoParaConectar[v] = pesoUV; // Atualiza o custo para conectar 'v'
                        paiNaMST[v] = u;               // 'u' se torna pai de 'v' na MST
                        pq.add(new Aresta(u, v, custoParaConectar[v])); // Adiciona/atualiza 'v' na PQ
                    }
                }
            }
            System.out.println(INDENT + "--------------------------------------------------------------------------------------------------------------------");
            System.out.println(INDENT + "Resultado Final:");
            System.out.println(INDENT + "MST: " + formatArestaList(mst));
            System.out.println(INDENT + "Custo Total da MST: " + custoTotal);
        }
    }

    /**
     * Demonstra operações fundamentais de uma Max-Heap.
     * Max-Heap: Árvore binária (geralmente completa) onde o valor de cada nó é maior ou igual aos valores de seus filhos. O maior elemento está na raiz.
     * Usada para implementar Filas de Prioridade (Max-Priority Queue).
     */
    static class OperacoesHeap {

        static class MaxHeap {
            private List<Integer> heap; // Armazena o heap como uma lista (representação de array)
            private String log = "";    // Para registrar passos da execução

            public MaxHeap() {
                heap = new ArrayList<>();
            }

            // Funções auxiliares para navegação na árvore (baseado em índice 0)
            private int parent(int i) { return (i - 1) / 2; }
            private int left(int i) { return 2 * i + 1; }
            private int right(int i) { return 2 * i + 2; }

            private void swap(int i, int j) {
                log += String.format(INDENT + INDENT + "Trocando heap[%d](%d) com heap[%d](%d)\n", i, heap.get(i), j, heap.get(j));
                int temp = heap.get(i);
                heap.set(i, heap.get(j));
                heap.set(j, temp);
            }

            /**
             * MAX-HEAPIFY: Mantém a propriedade de Max-Heap.
             * Assume que as subárvores rooted em left(i) e right(i) são Max-Heaps, mas A[i] pode ser menor que seus filhos. "Afunda" A[i] na heap até que a propriedade seja restaurada.
             * Complexidade: O(log N) ou O(altura da heap).
             */
            private void maxHeapify(int i) {
                log += String.format(INDENT + "maxHeapify(%d) em %s\n", i, heap);
                int l = left(i);
                int r = right(i);
                int largest = i; // Assume que o maior é a raiz da subárvore atual

                // Compara com o filho esquerdo
                if (l < heap.size() && heap.get(l) > heap.get(largest)) {
                    largest = l;
                }
                // Compara com o filho direito
                if (r < heap.size() && heap.get(r) > heap.get(largest)) {
                    largest = r;
                }
                // Se o maior não for a raiz atual 'i'
                if (largest != i) {
                    swap(i, largest); // Troca com o maior filho
                    maxHeapify(largest); // Chama recursivamente para a subárvore afetada
                }
            }

            /**
             * MAX-HEAP-INSERT: Insere uma nova chave na Max-Heap.
             * Adiciona a chave no final da heap (como uma nova folha). Depois, "sobe" a chave na árvore até que a propriedade de Max-Heap seja satisfeita.
             * Isso é feito conceitualmente chamando HEAP-INCREASE-KEY. Complexidade: O(log N).
             */
            public void insert(int key) {
                log = INDENT + "MAX-HEAP-INSERT(" + key + ")\n";
                // Adiciona a nova chave no final (aumenta o tamanho do heap)
                heap.add(key);
                log += String.format(INDENT + INDENT + "Chave %d adicionada ao final. Heap: %s\n", key, heap);

                // "Sobe" o elemento para sua posição correta (similar a HEAP-INCREASE-KEY)
                int i = heap.size() - 1;
                // Enquanto 'i' não for a raiz e o pai de 'i' for menor que 'i'
                while (i > 0 && heap.get(parent(i)) < heap.get(i)) {
                    log += String.format(INDENT + INDENT + "heap[parent(%d)] (%d) < heap[%d] (%d). Trocando.\n", i, heap.get(parent(i)), i, heap.get(i));
                    swap(i, parent(i));
                    i = parent(i); // Move para a posição do pai
                }
                System.out.print(log);
                System.out.println(INDENT + "Heap após insert: " + heap);
            }

            /**
             * HEAP-MAXIMUM: Retorna o maior elemento da Max-Heap. O maior elemento está sempre na raiz (índice 0).
             * Complexidade: O(1).
             */
            public int heapMaximum() {
                if (heap.isEmpty()) throw new IllegalStateException("Heap vazio");
                return heap.get(0);
            }

            /**
             * HEAP-EXTRACT-MAX: Remove e retorna o maior elemento da Max-Heap.
             * 1. Salva o elemento da raiz (o máximo).
             * 2. Move o último elemento da heap para a raiz.
             * 3. Diminui o tamanho da heap.
             * 4. Chama MAX-HEAPIFY na raiz para restaurar a propriedade de Max-Heap.
             * Complexidade: O(log N) devido ao MAX-HEAPIFY.
             */
            public int heapExtractMax() {
                log = INDENT + "HEAP-EXTRACT-MAX()\n";
                if (heap.isEmpty()) {
                    log += INDENT + INDENT + "Erro: Heap underflow\n";
                    System.out.print(log);
                    throw new IllegalStateException("Heap underflow");
                }
                log += String.format(INDENT + INDENT + "Heap antes: %s\n", heap);
                int max = heap.get(0);
                log += String.format(INDENT + INDENT + "Máximo é %d.\n", max);
                heap.set(0, heap.get(heap.size() - 1)); // Move o último para a raiz
                log += String.format(INDENT + INDENT + "Movendo último elemento (%d) para a raiz. Heap: %s\n", heap.get(heap.size()-1), heap);
                heap.remove(heap.size() - 1); // Remove o último
                if (!heap.isEmpty()) { // Só chama heapify se a heap não ficou vazia
                    log += String.format(INDENT + INDENT + "Removendo último. Heap agora: %s. Chamando maxHeapify(0).\n", heap);
                    maxHeapify(0); // Restaura a propriedade
                } else {
                    log += String.format(INDENT + INDENT + "Heap ficou vazio após remoção.\n");
                }
                System.out.print(log);
                System.out.println(INDENT + "Heap após extractMax: " + heap + ". Retornando " + max);
                return max;
            }

            /**
             * HEAP-INCREASE-KEY: Aumenta o valor de uma chave em uma determinada posição 'i'. Se a nova chave for menor que a atual, um erro ocorre (não demonstrado aqui como exceção).
             * Após atualizar A[i], a propriedade de Max-Heap pode ser violada se A[i] > A[PARENT(i)]. Então, a chave "sobe" na árvore até que a propriedade seja restaurada.
             * Complexidade: O(log N).
             */
            public void heapIncreaseKey(int i, int key) {
                log = INDENT + "HEAP-INCREASE-KEY(índice=" + i + ", novaChave=" + key + ")\n";
                if (i >= heap.size() || i < 0) {
                    log += INDENT + INDENT + "Erro: Índice inválido.\n";
                    System.out.print(log);
                    return;
                }
                if (key < heap.get(i)) {
                    log += INDENT + INDENT + "Erro: Nova chave ("+key+") é menor que a chave atual ("+heap.get(i)+").\n";
                    // System.out.print(log); // Opcional: mostrar o log mesmo em erro
                    // return; // No Cormen, isso é um erro.
                }
                heap.set(i, key);
                log += String.format(INDENT + INDENT + "Definido heap[%d]=%d. Heap: %s\n", i, key, heap);
                // "Sobe" o elemento se necessário
                while (i > 0 && heap.get(parent(i)) < heap.get(i)) {
                    log += String.format(INDENT + INDENT + "heap[parent(%d)] (%d) < heap[%d] (%d). ", i, heap.get(parent(i)), i, heap.get(i));
                    swap(i, parent(i));
                    log += String.format(INDENT + INDENT + "Heap após troca: %s\n", heap);
                    i = parent(i);
                }
                System.out.print(log);
                System.out.println(INDENT + "Heap após increaseKey: " + heap);
            }

            /**
             * BUILD-MAX-HEAP: Constrói uma Max-Heap a partir de um array desordenado.
             * Aplica MAX-HEAPIFY a todos os nós internos da árvore, começando da metade do array, (último nó pai) e indo em direção à raiz.
             * Complexidade: O(N) - parece O(N log N) mas uma análise mais apertada mostra que é linear.
             */
            public void buildMaxHeap(int[] arr) {
                heap.clear();
                for (int x : arr) heap.add(x);
                log = INDENT + "BUILD-MAX-HEAP a partir de " + Arrays.toString(arr) + "\n";
                log += String.format(INDENT + INDENT + "Heap inicial (como array): %s\n", heap);
                // Começa do último nó pai e vai até a raiz (índice 0)
                for (int i = (heap.size() / 2) - 1; i >= 0; i--) {
                    maxHeapify(i);
                }
                System.out.print(log);
                System.out.println(INDENT + "Heap após buildMaxHeap: " + heap);
            }
        }

        public static void demonstrarOperacoesMaxHeap(Scanner scanner) {
            System.out.println("\n### Demonstração: Operações de Max-Heap ###");
            System.out.println("Demonstra BUILD-MAX-HEAP, HEAP-MAXIMUM, HEAP-EXTRACT-MAX, HEAP-INCREASE-KEY, MAX-HEAP-INSERT.\n");

            MaxHeap heap = new MaxHeap();
            int[] initialElements = {4, 1, 3, 2, 16, 9, 10, 14, 8, 7};
            System.out.println(INDENT + "Construindo Max-Heap com elementos: " + Arrays.toString(initialElements));
            heap.buildMaxHeap(initialElements); // Constrói o heap
            System.out.println(SEPARADOR_LINHA);

            System.out.println(INDENT + "HEAP-MAXIMUM(): " + heap.heapMaximum()); // Mostra o máximo
            System.out.println(SEPARADOR_LINHA);

            System.out.println(INDENT + "Executando HEAP-EXTRACT-MAX():");
            int max = heap.heapExtractMax(); // Extrai o máximo
            System.out.println(INDENT + "Máximo extraído: " + max);
            System.out.println(SEPARADOR_LINHA);

            if (heap.heap.size() > 3) { // Verifica se o índice é válido após extração
                System.out.println(INDENT + "Executando HEAP-INCREASE-KEY(índice=3, novaChave=15): (índice 3 atual: " + heap.heap.get(3) + ")");
                heap.heapIncreaseKey(3, 15);
            } else {
                System.out.println(INDENT + "Heap muito pequeno para HEAP-INCREASE-KEY(3,15) após extração.");
            }
            System.out.println(SEPARADOR_LINHA);

            System.out.println(INDENT + "Executando MAX-HEAP-INSERT(chave=20):");
            heap.insert(20); // Insere novo elemento
            System.out.println(SEPARADOR_LINHA);

            System.out.println(INDENT + "Heap final: " + heap.heap);
        }
    }

    private static String formatArray(boolean[] arr) {
        StringBuilder sb = new StringBuilder("[");
        for(int i=0; i<arr.length; i++) sb.append(arr[i]?"V":"F").append(i==arr.length-1?"":",");
        sb.append("]");
        return sb.toString();
    }
    private static String formatDistPred(int[] arr, Map<Integer, String> nomes, int n) {
        StringBuilder sb = new StringBuilder("[");
        for(int i=0; i<n; i++){
            String valStr;
            if (arr[i] == Integer.MAX_VALUE) valStr = "Inf";
            else if (arr[i] == -1 && nomes != null) valStr = "NIL";
            else if (arr[i] == -1) valStr = "NIL";
            else valStr = nomes != null && nomes.containsKey(arr[i]) ? nomes.get(arr[i]) : String.valueOf(arr[i]);

            String keyStr = nomes != null && nomes.containsKey(i) ? nomes.get(i) : String.valueOf(i);
            sb.append(keyStr).append(":").append(valStr);
            if(i < n-1) sb.append(", ");
        }
        sb.append("]");
        return sb.toString();
    }
    private static String formatPQ(PriorityQueue<Aresta> pq) {
        if (pq.isEmpty()) return "[]";
        StringBuilder sb = new StringBuilder("[");
        PriorityQueue<Aresta> tempPQ = new PriorityQueue<>(pq); // Copia para não modificar a original
        while(!tempPQ.isEmpty()){
            Aresta a = tempPQ.poll();
            sb.append("(").append(a.destino).append(",").append(a.peso).append(")"); // Mostra (vértice, custo_para_alcançar_vértice)
            if(!tempPQ.isEmpty()) sb.append(", ");
        }
        sb.append("]");
        return sb.toString();
    }
    private static String formatStack(Stack<Integer> stack){
        if(stack.isEmpty()) return "[]";
        return stack.toString();
    }
    private static String formatQueue(Queue<Integer> queue){
        if(queue.isEmpty()) return "[]";
        return queue.toString();
    }
    private static String formatArestaList(List<Aresta> arestas) {
        if(arestas.isEmpty()) return "[]";
        StringBuilder sb = new StringBuilder("[");
        for(int i=0; i<arestas.size(); i++){
            sb.append(arestas.get(i).toString());
            if(i < arestas.size()-1) sb.append(", ");
        }
        sb.append("]");
        return sb.toString();
    }


    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);
        int escolha;

        do {
            System.out.println(SEPARADOR_LINHA);
            System.out.println("### Demonstrador Interativo de Algoritmos (Console Edition) ###");
            System.out.println("Escolha uma categoria de algoritmos para demonstrar:");
            System.out.println("\n--- Teoria dos Grafos ---");
            System.out.println("1. Representação de Grafo (Lista de Adjacência)");
            System.out.println("2. Representação de Grafo (Função de Incidência)");
            System.out.println("3. Busca em Profundidade (DFS)");
            System.out.println("4. Busca em Largura (BFS)");
            System.out.println("5. Algoritmo de Dijkstra");
            System.out.println("6. Algoritmo de Bellman-Ford");
            System.out.println("7. Algoritmo de Kruskal (MST)");
            System.out.println("8. Algoritmo de Prim (MST)");
            System.out.println("\n--- Estruturas de Dados ---");
            System.out.println("9. Operações de Max-Heap");
            System.out.println("\n0. Sair");
            System.out.print("\nSua escolha: ");

            try {
                if (scanner.hasNextInt()) {
                    escolha = scanner.nextInt();
                    scanner.nextLine();
                } else {
                    System.out.println("Entrada inválida. Por favor, insira um número.");
                    scanner.nextLine();
                    escolha = -1;
                    continue;
                }


                switch (escolha) {
                    case 1: RepresentacaoGrafo.demonstrarListaAdjacencia(scanner); break;
                    case 2: RepresentacaoGrafo.demonstrarFuncaoIncidencia(scanner); break;
                    case 3: BuscasGrafo.demonstrarDFS(scanner); break;
                    case 4: BuscasGrafo.demonstrarBFS(scanner); break;
                    case 5: CaminhosMinimos.demonstrarDijkstra(scanner); break;
                    case 6: CaminhosMinimos.demonstrarBellmanFord(scanner); break;
                    case 7: ArvoreGeradoraMinima.demonstrarKruskal(scanner); break;
                    case 8: ArvoreGeradoraMinima.demonstrarPrim(scanner); break;
                    case 9: OperacoesHeap.demonstrarOperacoesMaxHeap(scanner); break;
                    case 0: System.out.println("Saindo do demonstrador..."); break;
                    default: System.out.println("Opção inválida. Tente novamente.");
                }
            } catch (InputMismatchException e) {
                System.out.println("Entrada inválida. Por favor, insira um número.");
                scanner.nextLine();
                escolha = -1;
            } catch (Exception e) {
                System.err.println("Ocorreu um erro inesperado durante a demonstração:");
                escolha = -1;
            }

        } while (escolha != 0);

        scanner.close();
        System.out.println(SEPARADOR_LINHA);
        System.out.println("Demonstrador finalizado.");
    }
}