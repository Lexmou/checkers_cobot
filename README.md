"#checkers_cobot" 

# Checkers Cobot
## Mode Debug
Chaque fichier est fait pour être appelé depuis un environnement ROS. Ils peuvent tous prendre en argument :

- debug : informations de base
- debug full : informations complémentaires

## Fichiers Tests
- Le fichier test_gripper est relatif à la pinge Schunk WSG-50 lié au package wsg50-ros-pkg
- Le fichier test_kinematics est un test de reprise de la cinématique du module ur_kin_py dans sa version la plus récente
- Le fichier test_listener permet d'écouterce que le bras UR10 (package ROS/universal_robot avec les fichiers launch de ROS/ur_modern_driver en substitut de ROS/universal_robot/ur_driver)
- Le fichier test_move_custom permet de lancer des séquences tests de mouvements comme la réalisation de 3 coups du jeu de dame

## wsg50-ros-pkg

Ce  dossier contient deux fichiers launch légèrement modifiés. Celui contenant script est paramétrable depuis le terminal

##ur_kin_py

Ajoute simplement au code original la possibilité de retourner un tableau de solutions plutôt que la meilleure dans le cas de la cinématique inverse.
