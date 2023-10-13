
### Imagem do Docker
Link para o DockerHub: [sumo-plexe-jupyter](https://hub.docker.com/r/marcusfreire/sumo-plexe-jupyter)
```bash
docker pull marcusfreire/sumo-plexe-jupyter:latest
 ```

### Criando o Container
Um volume será compartilhado do container, que irá associar a uma pasta em seu computador, onde ficarão os códigos visualizados no jupyter notebook. Caminho no container: `/src/repository/`

Com o terminal na pasta que deseja abrir no seu computador digite o comando: 
* **Atenção!** Para que a janela do sumo-gui abra, há um pré-requisito: [Link configurar X11](#como-configurar-o-x11)

```bash
docker create -t -i --name givac -p 4000:4000 \
    -v ${PWD}:/src/repository/ \
    -e DISPLAY=$DISPLAY \
    -v /dev/dri:/dev/dri \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    sumo-plexe-jupyter:latest
 ```
 
 #### Iniciando o Container:
 ```bash
docker start givac
 ```
 * Agora no seu navegador acesse em [http://127.0.0.1:4000/](http://127.0.0.1:4000/) ou [http://localhost:4000](http://localhost:4000/)


#### Iniciando o Container no Docker Desktop Win
<img src="img/create_container_docker_desktop.png">

---
### Erros conhecidos:

 #### Como configurar o X11 
 * Permitir a visualização do SUMO-GUI fora do container erro no libGL

`FXApp::openDisplay: unable to open display :0.0`
 
O erro está relacionado à biblioteca gráfica `libGL`, que é usada por muitos programas para renderização gráfica, incluindo aplicações GUI como o SUMO-GUI. O erro indica que o Docker container está tentando acessar recursos gráficos que não estão disponíveis ou não são compatíveis com o ambiente dentro do container.

Para visualizar aplicações GUI de um container Docker no host Ubuntu, você precisa configurar o X11. Aqui está um guia passo a passo:

1. **Instale o X11 no Host (se ainda não estiver instalado)**:
   Se você ainda não tem o X11 instalado no seu sistema Ubuntu, instale-o:
   ```bash
   sudo apt-get install xorg openbox
   ```

2. **Permita o Acesso ao X11**:
   No host, execute o seguinte comando para permitir que qualquer usuário acesse o X11:
   ```bash
   xhost +
   ```
Erro apresentado caso não seja permitido o acesso:
`No protocol specified
FXApp::openDisplay: unable to open display :1`

   **Nota de Segurança**: Este comando permite que qualquer usuário se conecte ao seu servidor X11. Isso é útil para fins de teste, mas não é seguro para ambientes de produção. Em ambientes de produção, você deve restringir o acesso usando algo como `xhost local:docker` para permitir apenas conexões locais do usuário "docker".

3. **Configuração do Docker**:
   Ao executar o container Docker, você precisa mapear a variável de ambiente `DISPLAY` e o soquete X11. Isso permite que o container se comunique com o X11 do host.

   ```bash
   docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ubuntu-plexe-omnet-sumo-jupyter
   ```

   Aqui, `-e DISPLAY=$DISPLAY` define a variável de ambiente `DISPLAY` dentro do container para o mesmo valor que no host. E `-v /tmp/.X11-unix:/tmp/.X11-unix` mapeia o soquete X11 do host para o container.

4. **Execute o SUMO-GUI**:
   Agora, dentro do container, você deve ser capaz de executar o `sumo-gui` e ver a interface gráfica no seu host Ubuntu.

5. **Reverta as Permissões do X11 (quando terminar)**:
   Por razões de segurança, depois de terminar sua sessão Docker, é uma boa prática reverter as permissões do X11 para o estado anterior:
   ```bash
   xhost -
   ```

Lembre-se de que a configuração do X11 para permitir conexões externas pode ter implicações de segurança. Sempre tenha cuidado ao modificar as configurações de segurança e entenda os riscos associados.


### Permitindo que a janela do SUMO-GUI abra em outros sistemas operacionais:



#### 1. Windows:

**No Host**:

1. Instale o VcXsrv Windows X Server a partir de [https://sourceforge.net/projects/vcxsrv/](https://sourceforge.net/projects/vcxsrv/).
2. Inicie o XLaunch, escolha as configurações desejadas e na última página, marque "Disable access control".
3. Finalize a configuração e inicie o servidor X.

**No Dockerfile ou ao executar o container**:

1. Defina a variável de ambiente `DISPLAY` para o valor `host.docker.internal:0.0`.

#### 2. macOS:
**No Host**:

1. Instale o XQuartz a partir de [https://www.xquartz.org/](https://www.xquartz.org/).
2. Abra o XQuartz e vá para `Preferences > Security` e marque a opção "Allow connections from network clients".
3. Reinicie o XQuartz.
4. No terminal, execute `xhost +localhost`.

**No Dockerfile ou ao executar o container**:

1. Defina a variável de ambiente `DISPLAY` para o valor `host.docker.internal:0`.


---

### Erro de compartilhamento do arquivo amdgpu_dri.so:

```bash
 error: MESA-LOADER: failed to open amdgpu: /usr/lib/dri/amdgpu_dri.so: cannot open shared object file
 ```

 O erro indica que o Docker container está tentando acessar recursos gráficos que não estão disponíveis ou não são compatíveis com o ambiente dentro do container.

 Solução encontrada foi compartilhar volume:
```bash
docker run ... \
-v /dev/dri:/dev/dri
 ```
 Este diretório contém os drivers DRI (Direct Rendering Infrastructure) que são usados para permitir a renderização direta em sistemas Linux - O arquivo **`amdgpu_dri.so`** é o driver DRI para GPUs AMD.
