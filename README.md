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
  cp $STRAT_BASE/GoldobotStrat2020/build/output/Linux/Release/GoldobotStrat2020 $INSTALL
- de meme copier la conf dans $INSTALL:
  cp -r $STRAT_BASE/GoldobotStrat2020/conf $INSTALL
- editer les variables de l'environnement si necessaire: editer $INSTALL/conf/GoldobotStrat2020.yaml
- executer $INSTALL/GoldobotStrat2020
- pour generer le simulateur suivre les memes etapes mais a la place du "make" faire "make sim". Le nom de l'executable change aussi : GoldobotStrat2020_sim.


