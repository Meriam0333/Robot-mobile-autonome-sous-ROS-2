# Robot-mobile-autonome-sous-ROS-2
====================================================
Projet BE Robotique – Robot mobile autonome sous ROS 2
====================================================

Description :
Ce projet consiste en le développement d’un robot mobile autonome basé sur ROS 2.
Le travail est structuré en deux parties principales :
- une phase de simulation sous Gazebo (PC),
- une phase de déploiement et de validation sur robot réel (Raspberry Pi + Arduino).

----------------------------------------------------
Organisation du projet :
- ros2_ws_simulation/ : workspace ROS 2 utilisé pour la modélisation du robot,
  la simulation sous Gazebo et la validation des algorithmes de navigation.
- ros2_ws_embarque/   : workspace ROS 2 embarqué sur la Raspberry Pi pour
  l’exécution des missions autonomes sur le robot réel.
- arduino_firmware/   : code Arduino assurant la commande bas niveau des moteurs
  et la gestion des capteurs.
- rapport/            : rapport final du projet.

----------------------------------------------------
Prérequis :
- ROS 2 (Humble ou équivalent)
- Gazebo (pour la simulation)
- Python 3
- Arduino IDE (pour le firmware Arduino)
- Raspberry Pi OS (pour le déploiement embarqué)

----------------------------------------------------
Compilation (ROS 2) :
Dans chaque workspace ROS 2 :

  colcon build
  source install/setup.bash

----------------------------------------------------
Exécution (exemple) :
- Simulation :
  Lancer Gazebo et les nœuds ROS 2 depuis ros2_ws_simulation.

- Robot réel :
  Lancer les nœuds ROS 2 depuis ros2_ws_embarque sur la Raspberry Pi.
  Vérifier la connexion série avec l’Arduino avant l’exécution.

----------------------------------------------------
Remarques :
- Les dossiers build/, install/ et log/ sont générés automatiquement par colcon.
- Les paramètres de navigation peuvent être ajustés directement depuis le terminal.
- Le câblage matériel n’est pas détaillé ici, le cœur du projet portant sur
  l’architecture logicielle et les algorithmes de navigation.

Auteur : Meriam BANI , Abdellahi TFEIL , Aboubecrine cheikh BOUYE , Farah HENTATI
Année : 2525-2026

Rapport technique:
[ Télécharger le rapport ](https://drive.google.com/file/d/1jW24iy5zHKQ6aHnQ4ua7xWfTjyr0Pex4/view?usp=sharing)
