# GoldobotDetect2020
Rplidar experimental 2020

# Quick readme (in franch):
- il faut disposer d'un OS de type Linux (PC ou Raspi, Jetson devrait marcher aussi);
- cloner le projet github "GoldobotDetect2020". On va appeler le repertoire de la clone "$DETECT_BASE" par la suite;
- aller dans $DETECT_BASE/build;
- pour generer le programme de detection rplidar du robot executer "make";
- le binaire resultant se trouve dans :
  $DETECT_BASE/build/output/Linux/Release/;
- il s'apelle "GoldobotDetect2020";
- copier ce binaire dans votre repertoire d'installation (appelons le $INSTALL):
  $ cp $DETECT_BASE/build/output/Linux/Release/GoldobotDetect2020 $INSTALL
- de meme copier la conf dans $INSTALL:
  $ cp -r $DETECT_BASE/GoldobotDetect2020/conf $INSTALL
- editer les variables de l'environnement si necessaire: editer $INSTALL/conf/GoldobotDetect2020.yaml
- executer $INSTALL/GoldobotDetect2020

