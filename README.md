# 🦾 Projet Tiago Supermarket  

![Image du projet](https://github.com/hugopogo/tiago_supermarket/blob/main/images/tiago.png)



---

## 🔍 **Présentation du projet**  

Bonjour !  

Nous sommes **Hugo et Mathieu**, et voici notre projet **Supermarket Tiago** 🚀  

Ce projet utilise **ROS2** et **Tiago** pour détecter, attraper et transporter des canettes jusqu’à une table marquée par un **tag ArUco**.  

🔹 **Détection d’objets** : Identification des canettes avec OpenCV  
🔹 **Détection ArUco** : Repérage de la table cible  
🔹 **Navigation et manipulation** : Déplacement du robot et prise/dépôt des objets  

---

## 🎯 **Objectifs du projet**  

L'objectif initial était de **permettre à Tiago de récupérer une canette et de l’amener sur une table détectée via un marqueur ArUco**.  

Les étapes prévues étaient :  
1. 🥤 **Détection des canettes et navigation vers elles**  
2. ✋ **Saisie de l’objet**  
3. 🏃 **Navigation jusqu'à la table en détectant le tag ArUco**  
4. 🎯 **Alignement et préparation pour le dépôt**  
5. ✅ **Dépôt de la canette sur la table**  

---

## ✅ **Objectifs réalisés**  

✔️ Détection des canettes avec OpenCV  
✔️ Détection des marqueurs ArUco  
✔️ Déplacement et navigation autonome  
✔️ Préparation du bras pour la préhension des objets  

---

## ❌ **Objectifs non réalisés / à améliorer** (par manque de temps)  

❌ **Finalisation de la préhension des canettes**  
❌ **Optimisation du mouvement du bras pour éviter les collisions**  
❌ **Meilleure gestion des obstacles pendant la navigation**  
❌ **Ajout d'une détection dynamique des objets en mouvement**  

---

## 📌 Compilation et configuration du workspace

cd ~/ros2_ws
colcon build
source install/setup.bash

## 🎯 Lancement du projet (3 terminaux nécessaires)

Ouvre 3 terminaux et exécute les commandes suivantes :
1️⃣ Lancer la tâche principale

ros2 launch tiago_supermarket tiago_task.launch.py

2️⃣ Démarrer la simulation Tiago avec le supermarché

ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=supermarket

3️⃣ Activer MoveIt! pour la manipulation

ros2 launch tiago_moveit_config moveit_rviz.launch.py

## 🔗 Liens utiles

📌 Documentation officielle ROS2 → ROS2 Documentation
📌 Tutoriel sur la détection ArUco → https://gitlab.com/-/snippets/4826671
📌 Simulation Tiago sous Gazebo → https://gitlab.com/f2m2robserv/jazzy-ros-ynov/-/blob/main/README.md
📌 MoveIt! pour la manipulation du bras → MoveIt! Officiel
📌 Détection des canettes avec OpenCV → Cours et TP vus en classe
