# Strat
Couche de strategie (integrant la gestion des capteurs de "haut niveau" comme le rplidar et la camera)

# Quick readme (in franch):
- il faut disposer d'un OS de type Linux (PC ou Raspi, Jetson devrait marcher aussi);
- cloner le projet github "Strat". On va appeler le repertoire de la clone "$STRAT_BASE" par la suite;
- le programme de strat 2020 se trouve dans $STRAT_BASE/GoldobotStrat2020;
- aller dans $STRAT_BASE/GoldobotStrat2020/build;
- pour generer le programme du robot executer "make";
- le binaire resultant se trouve dans :
  $STRAT_BASE/GoldobotStrat2020/build/output/Linux/Release/;
- il s'apelle "GoldobotStrat2020";
- copier ce binaire dans votre repertoire d'installation (appelons le $INSTALL):
  $ cp $STRAT_BASE/GoldobotStrat2020/build/output/Linux/Release/GoldobotStrat2020 $INSTALL
- de meme copier la conf dans $INSTALL:
  $ cp -r $STRAT_BASE/GoldobotStrat2020/conf $INSTALL
- editer les variables de l'environnement si necessaire: editer $INSTALL/conf/GoldobotStrat2020.yaml
- executer $INSTALL/GoldobotStrat2020
- pour generer le simulateur suivre les memes etapes mais a la place du "make" faire "make sim". Le nom de l'executable change aussi : GoldobotStrat2020_sim.
- pour generer une version encore plus "realiste" du simulateur, il faut integrer le code d'un autre projet de "goldobot", a savoir le projet "goldo_robot_simulator".
- pour ceci il faut donc cloner ce projet:
  $ git clone https://github.com/goldobot/goldo_robot_simulator.git
- on va appeler le repertoire de base de "goldo_robot_simulator" : $GOLDO_ROBOT_SIMULATOR_DIR.
- revenir dans le repertoire $STRAT_BASE/GoldobotStrat2020/build et executer :
  $ make sim_ros EXTERNAL_PRJ_PATH=$GOLDO_ROBOT_SIMULATOR_DIR
  Le nom de l'executable genere sera comme dans le cas precedent : GoldobotStrat2020_sim.

# On a aussi un tutoriel. Quick readme pour ce tutoriel:
- il se decline en plusieurs etapes qui sont:
  - TUTO0 : juste un projet vide (mais qui compile!). Ce projet contient uniquement les classes "d'interface" necessaires pour la com avec les autres elements du robot (p.ex. la carte nucleo). Ca peut servir de "point de depart" pour une reimplementation "from scratch" de la strategie.
  - TUTO1 : illustration de l'api de lecture des fichiers de conf yaml.
    - Le code a regarder se trouve dans : $STRAT_BASE/GoldobotStrat2020/src/tuto/robot_strat_tuto1.cpp et il est "encadre" par le commentaire "TUTO1"
    - Compilation :
      $ cd $STRAT_BASE/GoldobotStrat2020/build ; make sim_tuto1
      $ cp $STRAT_BASE/GoldobotStrat2020/build/output/Linux/Release/GoldobotStrat2020_sim_tuto1 $INSTALL
    - Installation de la conf de base:
      $ cp -r $STRAT_BASE/GoldobotStrat2020/conf $INSTALL
    - Installation de l'example yaml:
      $ cp -f $STRAT_BASE/GoldobotStrat2020/src/tuto/strat_tuto1.yaml $INSTALL/strat.yaml
    - Lancer GoldobotStrat2020_sim_tuto1 dans le repertoire $INSTALL. Avoir sous les yeux le code de demo de robot_strat_tuto1.cpp et regarder les logs a l'execution pour apprendre a generer soi-meme ses fichiers de conf yaml.
  - TUTO2 : illustration de l'api de lecture de la position du robot et de l'api de detection des adversaires.
    - Le code a regarder se trouve dans : $STRAT_BASE/GoldobotStrat2020/src/tuto/robot_strat_tuto2.cpp et il est "encadre" par le commentaire "TUTO2"
    - Compilation :
      $ cd $STRAT_BASE/GoldobotStrat2020/build ; make sim_tuto2
      $ cp $STRAT_BASE/GoldobotStrat2020/build/output/Linux/Release/GoldobotStrat2020_sim_tuto2 $INSTALL
    - Installation de la conf de base:
      $ cp -r $STRAT_BASE/GoldobotStrat2020/conf $INSTALL
    - Lancer GoldobotStrat2020_sim_tuto2 dans le repertoire $INSTALL. Demarrer l'IHM et appuyer sur le boutton "Debug Strat GO!". Le simulateur affichera a des intervalles de 10 sec la position du robot (qui reste immobile..) et celle des adversaires.
  - TUTO3 : montre comment detecter le debut du match (tirette "reele" ou tirette "virtuelle" i.e. boutton "Debug Strat GO!") et comment envoyer une commande simple (une trajectoire en ligne droite).
    - Le code a regarder se trouve dans : $STRAT_BASE/GoldobotStrat2020/src/tuto/robot_strat_tuto3.cpp et il est "encadre" par le commentaire "TUTO3"
    - Compilation :
      $ make sim_tuto3 EXTERNAL_PRJ_PATH=$GOLDO_ROBOT_SIMULATOR_DIR
      $ cp $STRAT_BASE/GoldobotStrat2020/build/output/Linux/Release/GoldobotStrat2020_sim_tuto3 $INSTALL
    - Installation de la conf de base:
      $ cp -r $STRAT_BASE/GoldobotStrat2020/conf $INSTALL
    - Lancer GoldobotStrat2020_sim_tuto3 dans le repertoire $INSTALL. Demarrer l'IHM et appuyer sur le boutton "Debug Strat GO!". Dans l'affichage du terrain de l'IHM on peut voir notre robot se deplacer vers le centre du terrain (pendant que les adversaires se deplacent aussi..).
  - TUTO4 : montre comment detecter la fin d'une action et comment construire une machine a etats simple pour gerer une sequence d'actions.
    - Le code a regarder se trouve dans : $STRAT_BASE/GoldobotStrat2020/src/tuto/robot_strat_tuto4.{h|c}pp et il est "encadre" par le commentaire "TUTO4"
    - Compilation :
      $ make sim_tuto4 EXTERNAL_PRJ_PATH=$GOLDO_ROBOT_SIMULATOR_DIR
      $ cp $STRAT_BASE/GoldobotStrat2020/build/output/Linux/Release/GoldobotStrat2020_sim_tuto4 $INSTALL
    - Installation de la conf de base:
      $ cp -r $STRAT_BASE/GoldobotStrat2020/conf $INSTALL
    - Lancer GoldobotStrat2020_sim_tuto4 dans le repertoire $INSTALL. Demarrer l'IHM et appuyer sur le boutton "Debug Strat GO!". Dans l'affichage du terrain de l'IHM on peut voir notre robot se deplacer vers le centre du terrain (pendant que les adversaires se deplacent aussi..). Apres la premiere trajectoire en ligne droite il execute une rotation a 90° vers sa droite.

# NOTES 2022:
  * Build sim hack 2022:
  make sim_hack2022 EXTERNAL_PRJ_PATH=/home/jlouis/src/robotik/coupe2022/goldorak/soft/thomas/Carte_GR_SW4STM32