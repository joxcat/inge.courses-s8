# DevOps CI / CD
## Part 1
### 1-1 Document your database container essentials: commands and Dockerfile.
1. Création Dockerfile
```dockerfile
# En se basant sur l'image alpine de postgres 14.1
FROM postgres:14.1-alpine

# On configure une BDD nommée `db` avec comme utilisateur par défaut `usr` qui a comme mdp `pwd`
ENV POSTGRES_DB=db \
   POSTGRES_USER=usr \
   POSTGRES_PASSWORD=pwd
```
2. Création network: `docker network create app-network`
2. Lancement adminer: `docker run --rm -p 8090:8080 --network app-network --name adminer -d adminer`
3. Build bdd: `docker build . -t devops-bdd`
4. Lancement bdd avec persistance des données: `docker run -d --rm --name devops-bdd --network app-network -p 5432:5432 -e POSTGRES_USER=secureusr -e POSTGRES_PASSWORD=securepass -v "$PWD/db-data:/var/lib/postgresql/data" devops-bdd`

### 1-2 Why do we need a multistage build? And explain each step of this dockerfile.
```dockerfile
# Récupération de maven définie en tant que target docker `myapp-build`
FROM maven:3.8.6-amazoncorretto-17 AS myapp-build
# Définition de la valeur par défaut de la variable d'env MYAPP_HOME en tant que `/opt/myapp`
ENV MYAPP_HOME /opt/myapp
# Définition du dossier courant en tant que dossier pointé par la variable $MYAPP_HOME
WORKDIR $MYAPP_HOME
# Copie des fichiers sources dans le dossier courant
COPY pom.xml .
COPY src ./src
# Création du package maven
RUN mvn package -DskipTests

# Run
# Récupération d'une image du jdk 17
FROM amazoncorretto:17
ENV MYAPP_HOME /opt/myapp
WORKDIR $MYAPP_HOME
# Copie depuis la target docker `myapp-build` les fichiers .jar du dossier $MYAPP_HOME/target dans le dossier $MYAPP_HOME en tant que l'unique fichier myapp.jar
COPY --from=myapp-build $MYAPP_HOME/target/*.jar $MYAPP_HOME/myapp.jar

# Définition du point d'entrée du container avec la commande pour lancer l'application
ENTRYPOINT java -jar myapp.jar
```
- Build avec `docker build -t devops-backend .`
- Lancement avec `docker run --rm --name devops-backend -p 8080:8080 --network app-network devops-backend`

### 1-3 Document docker-compose most important commands.
- `docker compose up -d --build`: lance la configuration, en tache de fond (-d) et rebuild les images (--build)
- `docker compose logs <nom container> -f`: affiche les logs du container et suis la sorties des logs au fur et a mesure
- `docker compose down`: pour arreter les containers

### 1-4 Document your docker-compose file.
```yaml
# Définie la version de la syntaxe du docker-compose sur 3.7
version: '3.7'

services:
  backend:
    # Dossier ou la Dockerfile est contenue
    build: ./backend/simpleapi
    # Nom Host du container 
    hostname: devops-backend
    # Network liant les differents containers
    networks:
      - app-network
    # A besoin du container database pour se lancer 
    depends_on:
      - database
  database:
    build: ./database
    hostname: devops-bdd
    networks:
      - app-network
  httpd:
    build: ./frontend
    # Expose le port 80 du container sur le port 8080 de la machine hôte 
    ports:
      - 8080:80
    networks:
      - app-network
    depends_on:
      - backend

networks:
  # Créer un réseau interne à docker nommé `app-network`
  app-network:
```

### 1-5 Document your publication commands and published images in dockerhub.
1. `docker login`: Connection au hub de docker.io
2. `docker tag part1-database blackksoulls/inge.courses-s8.devops.takima.bdd:1.0`: ajout du nom du repo docker hub et de la version
3. `docker push blackksoulls/inge.courses-s8.devops.takima.bdd:1.0`: envoi de l'image sur docker hub

## Part 2
### 2-1 What are testcontainers?
Ce sont des librairies Java qui permettent de lancer des containers de différents services pour tester.

### 2-2 Document your Github Actions configurations.
```yaml
name: CI devops 2023
on:
  #to begin you want to launch this job in main and develop
  push:
    # branches:
    #   - main
    #   - develop
    branches: master
  # And on each pull request
  pull_request:

jobs:
  test-backend: 
    # We use the docker image ubuntu 22.04
    runs-on: ubuntu-22.04
    steps:
     #checkout your github code using actions/checkout@v2.5.0
      - uses: actions/checkout@v3

     #do the same with another action (actions/setup-java@v3) that enable to setup jdk 17 with the temurin distribution
      - name: Set up JDK 17
        uses: actions/setup-java@v3
        with:
          distribution: 'temurin'
          java-version: '17'

     #finally build your app with the latest command
      - name: Build and test with Maven
        # Change the current directory to the Java project directory
        working-directory: devops/TP/backend/simpleapi
        # Check the project with maven
        run: mvn clean verify
```
