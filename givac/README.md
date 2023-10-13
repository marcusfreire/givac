## GERENCIADOR DE INTERSEÇÕES PARA VEÍCULOS AUTÔNOMOS CONECTADOS - GIVAC

1. Notebooks Jupyter -  Com a execução dos códigos + análises.

2. Contém o cenário e os códigos em python:
    
    * Subpastas dos cenários:
        * SET: Arquivos XML com a configuração do SUMO como rotas, net.xml com estruturas e interseções entre outros arquivos.
        * CODE: Arquivos Python com os códigos para simulação.
        * OUTPUT: Saídas padrões do sumo de cada simulação.
    
    2-1. Semáforo: tradicional
        
    * Semáforo tradicional em que as fases estão em tempos fixos de duração.

    2-2. Semáforo: controle adptativo - [SUMO - ACTUATED](https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#type_actuated)
    
    * O SUMO suporta controle de tráfego atuado baseado em lacunas. Este esquema de controle é comum na Alemanha e funciona prolongando as fases de tráfego sempre que um fluxo contínuo de tráfego é detectado. Ele muda para a próxima fase após detectar um intervalo de tempo suficiente entre veículos sucessivos. Isso permite uma melhor distribuição do tempo verde entre as fases e também afeta a duração do ciclo em resposta às condições dinâmicas de tráfego.

    2-3. Semáforo: <sup>1</sup> gerenciador de intersection para veículos autônomos

    * Uma estratégia de gerenciamento de interseção para veículos autônomos na circunstância veículo-infraestrutura. Todos os veículos devem ser totalmente autônomos e podem se comunicar com a unidade de gerenciamento de interseção para verificar a situação do tráfego.

    ##### Os cenários a seguir foram usados da seguinte publicação:

    * <sup>1</sup> [Intersection management for autonomous vehicles with vehicle-to-infrastructure communication](https://journals.plos.org/plosone/article?id=10.1371/journal.pone.0235644) publicado por Yuying Li,Qipeng Liu. 