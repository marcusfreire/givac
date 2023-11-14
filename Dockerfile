# Use a imagem base do Ubuntu 20.04
FROM ubuntu:20.04

LABEL maintainer.email="mfreire.e@gmail.com"
LABEL maintainer.name="Marcus Freire"

#Dependências do sistema
RUN apt-get update -y
RUN apt-get install -y python3-dev python3-pip build-essential

# Evitar perguntas durante a instalação do pacote
ENV DEBIAN_FRONTEND=noninteractive

#Instalando dependencias para o SUMO
RUN apt-get update -y
RUN apt-get install -y git wget gcc bison flex perl cmake python3 g++ \
            libfox-1.6-dev libxerces-c-dev libeigen3-dev\
            qt5-default libqt5opengl5-dev libgdal-dev libproj-dev libgl2ps-dev\
            swig default-jdk maven libwebkit2gtk-4.0-dev \ 
            libopenscenegraph-dev libosgearth-dev openscenegraph-plugin-osgearth \
            libavformat-dev libavcodec-dev libswscale-dev python-dev python-configparser x11-apps\
            && rm -rf /var/lib/apt/lists/*            

#SUMO-1.18
RUN mkdir /src && cd /src && \
        wget https://sourceforge.net/projects/sumo/files/sumo/version%201.18.0/sumo-src-1.18.0.tar.gz && \
        tar -xzf sumo-src-1.18.0.tar.gz && rm sumo-src-1.18.0.tar.gz

ENV SUMO_HOME="/src/sumo-1.18.0"

RUN cd ${SUMO_HOME} &&\
    mkdir build/cmake-build && cd build/cmake-build &&\
    cmake ../.. && make -j $(nproc)

ENV PATH="${SUMO_HOME}/bin:${PATH}" 

#OMNET++

RUN cd /src && \
        wget https://github.com/omnetpp/omnetpp/releases/download/omnetpp-5.6.2/omnetpp-5.6.2-src-linux.tgz &&\
        tar -xzf omnetpp-5.6.2-src-linux.tgz && rm omnetpp-5.6.2-src-linux.tgz

ENV PATH="/src/omnetpp-5.6.2/bin:${PATH}"

RUN cd /src/omnetpp-5.6.2/ &&\
    ./configure WITH_OSG=no WITH_OSGEARTH=no &&\
    make -j $(nproc) MODE=debug base && make -j $(nproc) MODE=release base

#Install Veins
RUN cd /src && \
        git clone -b veins-5.1 https://github.com/sommer/veins.git && \
        cd veins/ &&\
        ./configure && make -j $(nproc)

#Install Plexe
RUN cd /src && \
        git clone -b plexe-3.0 https://github.com/michele-segata/plexe.git &&\
        cd plexe/ && \
        ./configure --with-veins=../veins && make -j $(nproc)

#Install Plexe-pyapi
RUN cd /src && \
        git clone  https://github.com/michele-segata/plexe-pyapi.git && \
        cd plexe-pyapi/ && \
        pip3 install .

#Atualização do gerenciador de pacotes e jupyter notebook
RUN pip3 install --upgrade pip && \
    pip3 install jupyter 

#Pacotes do Python
COPY requirements.txt /requirements.txt 
#Instalação dos pacotes listados python
RUN cd / && pip3 install --upgrade pip && \
        pip3 install -r requirements.txt &&\
        rm requirements.txt

#Diretório do usuário docker
RUN mkdir -p /src/repository/
WORKDIR /src/repository/

EXPOSE 4000

ENV DISPLAY=host.docker.internal:0.0

VOLUME [ "/src/repository/" ]

CMD ["sh", "-c","jupyter notebook --ip=0.0.0.0 --port 4000 --allow-root --no-browser --NotebookApp.token=''"]