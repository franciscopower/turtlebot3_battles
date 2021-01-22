# Turtlebot3 Battles

*Um jogo da apanhada jogado por veículos autónomos.*

Este projeto contém vários programas diferentes, sendo o jogo da apanhada autónomo o principal. Consiste em 3 equipas, a azul, vermelha e verde, em que a equipa vermelha tem como objetivo caçar a verde, a verde caçar a azul e a azul caçar a vermelha. 

O projeto está organizado em 3 sub projetos: p_fpower_bringup, p_fpower_core e p_fpower_description.

## p_fpower_bringup
```bash
.
├── CMakeLists.txt
├── config
│   ├── game.yaml
│   ├── gmapping_params.yaml
│   └── visualize.rviz
├── launch
│   ├── bringup_gazebo.launch
│   ├── game.launch
│   ├── gmapping.launch
│   ├── spawn.launch
│   ├── teleop.launch
│   └── visualize.launch
└── package.xml
```
Este sub projeto contém todos os ficheiros .launch necessários assim como os ficheiros de configuração.
- **bringup_gazebo.launch** - Lança o gazebo com um determinado mundo.
- **game.launch** - Faz o *spawn* de 3 robôs com nomes e cores diferentes e 3 nós *player* do pacote *p_fpower_core*.
- **gmapping.launch** - Lança um nó de *slam* com determinados argumentos
- **spawn.launch** - faz o spawn de um robô, com os parametros default de *player_name:=p_fpower* e *player_color:=Green*.
- **teleop.launch** - lança o nó de teleop para controlar o robô usando o teclado.
- **visualize.launch** - lança o rviz como configurado com o ficheiro de configuração config/visualize.rviz.

## p_fpower_description
```bash
.
├── CMakeLists.txt
├── config
│   └── controller.yaml
├── package.xml
└── urdf
    ├── p_fpower.gazebo.macro.xacro
    ├── p_fpower.sdf
    ├── p_fpower.urdf
    ├── p_fpower.urdf.xacro
    ├── properties.xacro
    ├── ros_control.gazebo.xacro
    └── ros_diff_drive.gazebo.xacro
```
Contém todos os ficheiros de descrição do robô e controlador.

## p_fpower_core
```bash
.
├── CMakeLists.txt
├── nodes
│   ├── fair_play_player.py
│   ├── go_to_goal.py
│   ├── model_states_to_tf.py
│   ├── player.py
└── package.xml
```
Contém os scripts python que controlam o robô.

- **fair_play_player.py** - o robô persegue a equipa que pretende caçar, identifica colisões e, caso colida com a equipa presa, ganha um ponto, caso colida com a equipa caçadora perde um ponto e envia uma mensagem ao gazebo para ser reposicionada num local aleatório do mapa. A sua pontuação é constantemente publicada no topico /<player_name>/score
- **go_to_goal.py** - o robô desloca-se para uma *goal* selecionada no *rviz*
- **model_states_to_tf.py** - subscreve às mensagens de model_states e cria uma transformação global do world para cada odom do model state.
- **player.py**:
  - subscreve a mensagens de imagem e deteta os centroides do robô da equipa caçadora e presa mais próximos (maior área na imagem).
  - sbuscreve a mensagens de laser scan e deteta a presença de uma parede à sua frente.
  - envia mensagens de twist, calculadas tendo em consideração o angulo necessário para se direcionar na direço da presa menos o angulo necessário para se direcionar da direção contrária do caçador. As velocidades lineares de perseguição e fuga são de 1 m/s.
  - caso não detete nenhum caçador nem nenhuma presa, na presença de algum obstáculo o robô diminui a sua velocidade linear e publica uma velocidade angular de modo a girar 30º na direção contrária da parede. 

## Jogo da apanhada.

Para inicializar o jogo da apanhada, é necessário lançar 2 ficheiros launch e um nó.

```bash
$ roslaunch p_fpower_bringup bringup_gazebo.launch
$ roslaunch p_fpower_bringup game.launch
$ rosrun th_referee th_referee
```
O programa th_referee (desenvolivdo pelo professor Miguel Oliveira) arbitra o jogo. 
