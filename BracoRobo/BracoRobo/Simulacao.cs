/*
    Simulação do Braço Robô MRB-5GL - Simulates MRB-5GL, a 5 DOF Robot Arm prototype

    Copyright (C) 2019  Amaro Duarte de Paula Neto

    This file is part of Simulação do Braço Robô MRB-5GL.

    Simulação do Braço Robô MRB-5GL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Simulação do Braço Robô MRB-5GL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Simulação do Braço Robô MRB-5GL.  If not, see <https://www.gnu.org/licenses/>.

    contact e-mail: amaro.net80@gmail.com


    Este arquivo é parte do programa Simulação do Braço Robô MRB-5GL

    Simulação do Braço Robô MRB-5GL é um software livre; você pode redistribuí-lo e/ou
    modificá-lo dentro dos termos da Licença Pública Geral GNU como
    publicada pela Free Software Foundation (FSF); na versão 3 da
    Licença, ou (a seu critério) qualquer versão posterior.

    Simulação do Braço Robô MRB-5GL é distribuído na esperança de que possa ser útil,
    mas SEM NENHUMA GARANTIA; sem uma garantia implícita de ADEQUAÇÃO
    a qualquer MERCADO ou APLICAÇÃO EM PARTICULAR. Veja a
    Licença Pública Geral GNU para maiores detalhes.

    Você deve ter recebido uma cópia da Licença Pública Geral GNU junto
    com este programa, Se não, veja <http://www.gnu.org/licenses/>.
*/
using System;
//using System.Windows.Forms;
using System.Collections.Generic;
using System.IO.Ports;
using BracoRobo.Classes;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace BracoRobo
{
    /// <summary>
    /// Classe para simular o robô e o mundo onde ele está.
    /// </summary>
    public class SimulacaoDoRobo : Microsoft.Xna.Framework.Game
    {
        /// <summary>
        /// String para receber caracteres via porta serial. O acesso a esta string só deve ocorrer quando todos os caracteres
        /// forem recebidos.
        /// </summary>
        String bfRecebe = String.Empty;
        
        /// <summary>
        /// Objeto para a porta serial que irá receber os comandos do robô, bem como mandar para o controle
        /// quaisquer dados referentes ao robô que se fizerem necessários, tais como dados dos sensores, por exemplo.
        /// </summary>
        private SerialPort serialCom = new SerialPort();

        /// <summary>
        /// String que possui os comandos aplicados ao robô via porta serial. Esta string irá obter o valor do campo
        /// bfRecebe, quando este tiver recebido todos os caracteres que compõem os comandos do robô.
        /// </summary>
        String comandoSerial = String.Empty;

        /// <summary>
        /// Trata da configuração e do gerenciamento do dispositivo gráfico da simulação.
        /// </summary>
        GraphicsDeviceManager graphics;
        /// <summary>
        /// Variável a ser utilizada para o uso de texturas.
        /// </summary>
        SpriteBatch spriteBatch;

        /// <summary>
        /// Câmera que aponta para o braço robô
        /// </summary>
        Camera cameraBracoRobo;

        /// <summary>
        /// Câmera que aponta para a placa Easyled
        /// </summary>
        Camera cameraEasyled;
        /// <summary>
        /// Câmera que aponta para o pulso da garra
        /// </summary>
        Camera cameraPulsoGarra;
        /// <summary>
        /// Câmera que aponta para o ponto da garra
        /// </summary>
        Camera cameraPontoGarra;

        /// <summary>
        /// Variável que aponta para a câmera que está sendo usada no momento
        /// </summary>
        Camera cameraCorrente;

        /// <summary>
        /// Matriz que representa o mundo 3D. Basicamente, esta matriz, aplicada ao nosso objeto 3D, nos diz onde o objeto se localiza no
        /// referencial do mundo 3D.
        /// </summary>
        private Matrix world = Matrix.Identity;
        /// <summary>
        /// Matriz que representa a câmera. Nela estão definidas a posição da câmera, o ponto para onde está olhando e a orientação
        /// (em pé, de cabeça pra baixo, deitada etc.).
        /// </summary>
        private Matrix view = Matrix.CreateLookAt(new Vector3(0.0f, 20.0f, 20.0f), new Vector3(0, 0, 0), Vector3.UnitY);
        /// <summary>
        /// Matriz de Projeção. Define como o mundo 3D aparece na janela (ou na tela, se estiver em tela cheia).
        /// </summary>
        private Matrix projection = Matrix.CreatePerspectiveFieldOfView(MathHelper.ToRadians(45), 800f / 600f, 0.1f, 1000f);
        
        /// <summary>
        /// Vetor para indicar quais partes do braço robô estão habilitadas para visualização
        /// </summary>
        private bool[] habilitaParte;

        /// <summary>
        /// Variável para indexar o vetor habilitaParte
        /// </summary>
        private int posHabParte = 0;
        /// <summary>
        /// Variável para indicar se a mudança automática de câmera está ou não ativada
        /// </summary>
        private bool mudancaAutoDeCameraAtivada = false;

        /// <summary>
        /// Estado corrente do teclado
        /// </summary>
        KeyboardState keyboardState;

        /// <summary>
        /// Estado anterior do teclado
        /// </summary>
        KeyboardState keyboardStateAnt;

        /// <summary>
        /// Instância do robô
        /// </summary>
        Robo robo;

        /// <summary>
        /// Referencial cartesiano da base fixa
        /// </summary>
        ReferencialCartesiano referencialBaseFixa;
        /// <summary>
        /// Referencial cartesiano do ponto da garra
        /// </summary>
        ReferencialCartesiano referencialPontoGarra;
        /// <summary>
        /// Referencial cartesiano do ponto da garra calculado a partir dos parâmetros de Denavit-Hatenberg
        /// </summary>
        ReferencialCartesiano referencialPontoGarraDH;
                
        /// <summary>
        /// Plano do braço robô
        /// </summary>
        Plano planoBracoRobo;
        /// <summary>
        /// Plano que passa pelo ponto da garra
        /// </summary>
        Plano planoGarra;
        /// <summary>
        /// Plano que passa pela origem do referencial da base fixa e pelo ponto da garra
        /// </summary>
        Plano planoOrigemBaseFixaGarra;

        /// <summary>
        /// Parâmetros alfa de Denavit-Hatenberg
        /// </summary>
        float[] alfa;
        /// <summary>
        /// Parâmetros "a" de Denavit-Hatenberg
        /// </summary>
        float[] a;
        /// <summary>
        /// Parâmetros "d" de Denavit-Hatenberg. O índice 0 não é usado.
        /// </summary>
        float[] d;

        /// <summary>
        /// Parâmetros alfa de Denavit-Hatenberg (exatos)
        /// </summary>
        float[] alfaExato;
        /// <summary>
        /// Parâmetros "a" de Denavit-Hatenberg (exatos)
        /// </summary>
        float[] aExato;
        /// <summary>
        /// Parâmetros "d" de Denavit-Hatenberg (exatos). O índice 0 não é usado.
        /// </summary>
        float[] dExato;

        /// <summary>
        /// Parâmetros alfa de Denavit-Hatenberg (planificados)
        /// </summary>
        float[] alfaPlan;
        /// <summary>
        /// Parâmetros "a" de Denavit-Hatenberg (planificados)
        /// </summary>
        float[] aPlan;
        /// <summary>
        /// Parâmetros "d" de Denavit-Hatenberg (planificados). O índice 0 não é usado.
        /// </summary>
        float[] dPlan;

        /// <summary>
        /// Indica se os parâmetros DH exatos devem ser usados nos referenciais cartesianos dos segmentos do braço robô
        /// </summary>
        bool parametrosDHExatos = false;

        /// <summary>
        /// Matrizes de rotação em X de um ângulo alfa (parametro de Denavit-Hatenberg)
        /// </summary>
        Matrix[] RxAlfa;
        /// <summary>
        /// Matrizes de translação em X de uma distância a (parâmetro de Denavit-Hatenberg)
        /// </summary>
        Matrix[] Dxa;
        /// <summary>
        /// Matrizes de translação em Z de uma distância d (parâmetro de Denavit-Hatenberg)
        /// </summary>
        Matrix[] Dzd;

        /// <summary>
        /// Matrizes de rotação em X de um ângulo alfa (parametro de Denavit-Hatenberg planificado)
        /// </summary>
        Matrix[] RxAlfaPlan;
        /// <summary>
        /// Matrizes de translação em X de uma distância a (parâmetro de Denavit-Hatenberg planificado)
        /// </summary>
        Matrix[] DxaPlan;
        /// <summary>
        /// Matrizes de translação em Z de uma distância d (parâmetro de Denavit-Hatenberg planificado)
        /// </summary>
        Matrix[] DzdPlan;

        /// <summary>
        /// Referenciais cartesianos obtidos a partir dos parâmetros de Denavit-Hatenberg
        /// </summary>
        ReferencialCartesiano[] frame;
        /// <summary>
        /// Conjunto de vetores de projeção da posição do pulso da garra no plano vertical que corta o braço robô pela origem da base fixa
        /// </summary>
        ConjuntoVetoresProjecao conjuntoVetoresProjecao;
        /// <summary>
        /// String contendo as coordenadas da posição alvo da garra especificadas em angulos fixos
        /// </summary>
        String coordenadasXYZGamaBetaAlfa = String.Empty;
        
        /// <summary>
        /// Construtor padrão da simulação.
        /// </summary>
        public SimulacaoDoRobo(string[] args)
        {
            Sobre();
            Console.WriteLine("Pressione ENTER para continuar.");
            Console.ReadLine();

            habilitaParte = new bool[12];

            for (int i = 0; i < habilitaParte.Length; i++)
            {
                habilitaParte[i] = true;
            }

            cameraBracoRobo = new Camera();
            cameraEasyled = new Camera();
            cameraPulsoGarra = new Camera();
            cameraPontoGarra = new Camera();

            cameraBracoRobo.posicaoApontadaInicial = new Vector3(0.0f, 26.0f, 0.0f);
            cameraBracoRobo.distanciaCameraInicial = 75;
            cameraBracoRobo.azimuteCameraInicial = 25;
            cameraBracoRobo.elevacaoCameraInicial = 45;

            cameraBracoRobo.ResetaPosicao();

            cameraEasyled.posicaoApontadaInicial = new Vector3(6.15f, 1.35f, 29.711f);            
            cameraEasyled.distanciaCameraInicial = 12;
            cameraEasyled.azimuteCameraInicial = 163;
            cameraEasyled.elevacaoCameraInicial = 45;

            cameraEasyled.ResetaPosicao();

            cameraPulsoGarra.posicaoApontadaInicial = new Vector3(0.0f, 26.0f, 0.0f);
            cameraPulsoGarra.distanciaCameraInicial = 45;
            cameraPulsoGarra.azimuteCameraInicial = 25;
            cameraPulsoGarra.elevacaoCameraInicial = 45;

            cameraPulsoGarra.ResetaPosicao();

            cameraPontoGarra.posicaoApontadaInicial = new Vector3(0.0f, 26.0f, 0.0f);
            cameraPontoGarra.distanciaCameraInicial = 45;
            cameraPontoGarra.azimuteCameraInicial = 25;
            cameraPontoGarra.elevacaoCameraInicial = 45;

            cameraPontoGarra.ResetaPosicao();

            // Inicia com a câmera apontada para o braco robô
            cameraCorrente = cameraBracoRobo;

            Console.WriteLine("Construindo simulação...");
            graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";

            robo = new Robo(args);

            robo.ComandoLEDRecebido += new ComandoLEDRecebidoEventHandler(robo_ComandoLEDRecebido);
            robo.ComandoAcionamentoServoRecebido += new ComandoAcionamentoServoRecebidoEventHandler(robo_ComandoAcionamentoServoRecebido);

            alfaExato = new float[]{0f, 90f, 0f, 0f, 90f};
            aExato = new float[]{0f, 0.025f, 11.65f, 5.825f, 0.45f};
            dExato = new float[]{0f, 17.41098f, 4.293516f, -3.438032f, -2.174584f, 8.633297f};

            alfaPlan = new float[] {0f, 90f, 0f, 0f, 90f};
            aPlan = new float[] {0f, 0f, 11.65f, 5.825f, 0f};
            dPlan = new float[] {0f, 17.41098f, 0f, 0f, 0f, 0f};

            RxAlfa = new Matrix[5];
            Dxa = new Matrix[5];
            Dzd = new Matrix[6];

            RxAlfaPlan = new Matrix[5];
            DxaPlan = new Matrix[5];
            DzdPlan = new Matrix[6];            

            referencialBaseFixa = new ReferencialCartesiano();
            referencialPontoGarra = new ReferencialCartesiano();
            referencialPontoGarraDH = new ReferencialCartesiano();

            frame = new ReferencialCartesiano[6];

            frame[0] = referencialBaseFixa;
            for (int i = 1; i < 6; i++)
                frame[i] = new ReferencialCartesiano();

            referencialBaseFixa.TamanhoEixoX = 14.0f;
            referencialBaseFixa.posicaoLetraX.Z = -15.5f;

            referencialBaseFixa.TamanhoEixoY = 12.0f;
            referencialBaseFixa.posicaoLetraY.X = -13.5f;
            
            referencialBaseFixa.TamanhoEixoZ = 3.0f;
            referencialBaseFixa.posicaoLetraZ.Y = 3.0f;
            referencialBaseFixa.posicaoLetraZ.X = -1.0f;

            frame[1].TamanhoEixoZ = 7.0f;
            frame[1].posicaoLetraZ.Y = 8.8f;

            conjuntoVetoresProjecao = new ConjuntoVetoresProjecao();
            //conjuntoVetoresProjecao.TamanhoVetorK = 0.5f;
            //conjuntoVetoresProjecao.tamanhoPontaSetaVetorK = 0.5f;
            //conjuntoVetoresProjecao.posicaoLetraK.Z = 1.0f;

            AlternaParametrosDH();

            for (int i = 1; i <= alfa.Length; i++)
            {
                RxAlfaPlan[i - 1] = Matrix.CreateRotationZ(MathHelper.ToRadians(-alfaPlan[i - 1]));
                DxaPlan[i - 1] = Matrix.CreateTranslation(0.0f, 0.0f, -aPlan[i - 1]);
                DzdPlan[i] = Matrix.CreateTranslation(0.0f, dPlan[i], 0.0f);
            }

            planoBracoRobo = new Plano();
            planoGarra = new Plano();
            planoOrigemBaseFixaGarra = new Plano();

            Console.WriteLine("Construindo simulação...OK");

            if (mudancaAutoDeCameraAtivada)
                Console.WriteLine("Mudança automática de câmera ativada. Shift + L para desativar.");
            else
                Console.WriteLine("Mudança automática de câmera desativada. Shift + L para ativar.");
            Console.WriteLine("L para alternar entre o braço robô e a placa Easyled");
            Console.WriteLine("F1 para ajuda dos demais comandos de teclado.");
        }

        /// <summary>
        /// Calcula/Recalcula as matrizes dos parâmetros fixos de Denavit-Hatenberg.
        /// Neste caso, os parâmetros fixos são os alfas, os a's e os d's.
        /// </summary>
        private void CalculaMatrizesDHFixas()
        {
            for (int i = 1; i <= alfa.Length; i++)
            {
                RxAlfa[i - 1] = Matrix.CreateRotationZ(MathHelper.ToRadians(-alfa[i - 1]));
                Dxa[i - 1] = Matrix.CreateTranslation(0.0f, 0.0f, -a[i - 1]);
                Dzd[i] = Matrix.CreateTranslation(0.0f, d[i], 0.0f);
            }
        }

        /// <summary>
        /// Alterna os parâmetros DH entre os exatos e os planificados
        /// </summary>
        private void AlternaParametrosDH()
        {
            if (parametrosDHExatos)
            {
                alfa = alfaExato;
                a = aExato;
                d = dExato;

                frame[2].TamanhoEixoX = frame[2].tamanhoEixoXOriginal;
                frame[2].posicaoLetraX.Z = frame[2].posicaoLetraXOriginal.Z;

                frame[4].TamanhoEixoX = frame[4].tamanhoEixoXOriginal;
                frame[4].posicaoLetraX.Z = frame[4].posicaoLetraXOriginal.Z;
                frame[4].TamanhoEixoZ = frame[4].tamanhoEixoZOriginal;
                frame[4].posicaoLetraZ.Y = frame[4].posicaoLetraZOriginal.Y;

                frame[5].TamanhoEixoZ = frame[5].tamanhoEixoZOriginal;
                frame[5].posicaoLetraZ.Y = frame[5].posicaoLetraZOriginal.Y;                

                Console.WriteLine("Parâmetros DH exatos.");
            }
            else
            {
                alfa = alfaPlan;
                a = aPlan;
                d = dPlan;

                frame[2].TamanhoEixoX = 5.0f;
                frame[2].posicaoLetraX.Z = -6.5f;

                frame[4].TamanhoEixoX = 7.0f;
                frame[4].posicaoLetraX.Z = -8.5f;
                frame[4].TamanhoEixoZ = 7.0f;
                frame[4].posicaoLetraZ.Y = 8.5f;

                frame[5].TamanhoEixoZ = 17f;
                frame[5].posicaoLetraZ.Y = 18.5f;                

                Console.WriteLine("Parâmetros DH planificados.");
            }

            CalculaMatrizesDHFixas();
        }

        /// <summary>
        /// Destrutor da simulação.
        /// </summary>
        ~SimulacaoDoRobo()
        {

        }

        /// <summary>
        /// Permite a simulação (jogo) fazer qualquer inicialização que precisar antes de 
        /// começar a rodar. É aqui que a simulação pode requisitar quaisquer serviços e
        /// e carregar qualquer conteúdo (content) não gráfico relacionado. Chamando
        /// base.Initialize() vai enumerar quaisquer componentes e inicializá-los também.
        /// </summary>
        protected override void Initialize()
        {
            Console.WriteLine("Inicializando...");
            base.Initialize();
            this.IsMouseVisible = true;
            this.graphics.IsFullScreen = false;
            Console.WriteLine("Inicializando...OK");
        }

        /// <summary>
        /// LoadContent será chamado uma vez por simulação (jogo) e é o lugar
        /// para carregar todo o seu conteúdo
        /// </summary>
        protected override void LoadContent()
        {
            /* LEMBRETE: AO EXPORTAR OS MODELOS 3D DO WINGS3D, LEMBRAR QUE O ARQUIVO EXPORTADO VIRÁ
               COM O EIXO X INVERTIDO. OU SEJA, SE O EIXO X NO WINGS3D APONTAR PARA A ESQUERDA, O 
               EIXO X DO ARQUIVO EXPORTADO APONTARÁ PARA A DIREITA */

            Console.WriteLine("Carregando conteúdo...");
            // Cria um novo SpriteBatch, que pode ser usado para desenhar texturas.
            spriteBatch = new SpriteBatch(GraphicsDevice);
            
            robo.baseFixa.modelo = Content.Load<Model>("base_fixa");
            robo.motorBase.modelo = Content.Load<Model>("servo1");
            robo.baseGiratoria.modelo = Content.Load<Model>("base_giratoria");
            robo.motor1.modelo = Content.Load<Model>("servo2");
            robo.bracoL1.modelo = Content.Load<Model>("segmento_L1");
            robo.motor2.modelo = Content.Load<Model>("servo1");
            robo.bracoL2.modelo = Content.Load<Model>("segmento_L2");
            robo.motor3.modelo = Content.Load<Model>("servo1");
            robo.bracoL3.modelo = Content.Load<Model>("segmento_L3");
            robo.motorGiroGarra.modelo = Content.Load<Model>("servo1");
            robo.garra.modelo = Content.Load<Model>("garra_MK2");
            robo.garra.servoDaGarra.modelo = Content.Load<Model>("servo3");
            referencialBaseFixa.modelo = Content.Load<Model>("eixos_cartesianos");
            referencialPontoGarra.modelo = Content.Load<Model>("eixos_cartesianosGarra");
            referencialPontoGarraDH.modelo = Content.Load<Model>("eixos_cartesianosGarra");

            for (int i = 1; i < 6; i++)
            {
                String nomeModeloEixoCartesiano = "eixos_cartesianos" + i.ToString();
                frame[i].modelo = Content.Load<Model>(nomeModeloEixoCartesiano);
            }

            planoBracoRobo.modelo = Content.Load<Model>("parte_plano_braco");
            planoGarra.modelo = Content.Load<Model>("parte_plano_garra");
            planoOrigemBaseFixaGarra.modelo = Content.Load<Model>("parte_plano_obliquo");

            conjuntoVetoresProjecao.modeloFixo = Content.Load<Model>("conjunto_vetores_projecao");
            conjuntoVetoresProjecao.modeloVetorZt = Content.Load<Model>("vetorZt");

            UpdateView();
            Console.WriteLine("Carregando conteúdo...OK");
        }

        /// <summary>
        /// UnloadContent será chamado uma vez por simulação (jogo) e é o lugar
        /// para desalocar (descarregar) todo o conteúdo.
        /// </summary>
        protected override void UnloadContent()
        {
            
        }

        /// <summary>
        /// Método executado ao ocorrer o evento de recebimento do comando LED pela classe braço robô
        /// </summary>
        /// <param name="sender">objeto que disparou o evento</param>
        /// <param name="e">argumentos do evento</param>
        private void robo_ComandoLEDRecebido(object sender, EventArgs e)
        {
            if (mudancaAutoDeCameraAtivada)
            {
                cameraCorrente = cameraEasyled;
                UpdateView();
            }
        }
        /// <summary>
        /// Método executado ao ocorrer o evento de recebimento de qualquer comando de acionamento
        /// dos servos (JST, RPS, CTZ, GA, GF)
        /// </summary>
        /// <param name="sender">objeto que disparou o evento</param>
        /// <param name="e">argumentos do evento</param>
        private void robo_ComandoAcionamentoServoRecebido(object sender, EventArgs e)
        {
            if (mudancaAutoDeCameraAtivada)
            {
                cameraCorrente = cameraBracoRobo;
                UpdateView();
            }
        }

        /// <summary>
        /// Extrai da matrix de transformação a sua componente de rotação, e a retorna em uma nova matriz.
        /// </summary>
        /// <param name="T">Matriz de transformação</param>
        /// <returns>Matriz contendo somente a componente de rotação de T</returns>
        private Matrix ExtraiMatrizRotacao(Matrix T)
        {
            Matrix R = Matrix.Identity;

            R.M11 = T.M11;
            R.M12 = T.M12;
            R.M13 = T.M13;

            R.M21 = T.M21;
            R.M22 = T.M22;
            R.M23 = T.M23;

            R.M31 = T.M31;
            R.M32 = T.M32;
            R.M33 = T.M33;

            return R;
        }

        /// <summary>
        /// Executa os comandos do teclado
        /// </summary>
        private void UpdateKeyboard()
        {
            keyboardStateAnt = keyboardState;
            keyboardState = Keyboard.GetState();            

            // Allows the game to exit
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed || keyboardState.IsKeyDown(Keys.Escape))
            {
                this.Exit();
            }

            ////// Câmera e suas coordenadas ////////

            // Câmera dos LEDs
            if (keyboardState.IsKeyDown(Keys.L) && !keyboardStateAnt.IsKeyDown(Keys.L))
            {
                // Shift + L ativa/desativa mudança automática de câmera
                if (keyboardState.IsKeyDown(Keys.LeftShift) || keyboardState.IsKeyDown(Keys.RightShift))
                {
                    if (mudancaAutoDeCameraAtivada)
                    {
                        mudancaAutoDeCameraAtivada = false;
                        Console.WriteLine("Mudança automática de câmera desativada. Shift + L para ativar.");
                    }
                    else
                    {
                        mudancaAutoDeCameraAtivada = true;
                        Console.WriteLine("Mudança automática de câmera ativada. Shift + L para desativar.");
                    }
                        
                }
                // L muda a câmera entre braço robô e placa Easyled
                else
                {
                    if (cameraCorrente != cameraEasyled)
                        cameraCorrente = cameraEasyled;
                    else
                        cameraCorrente = cameraBracoRobo;
                }
                
                UpdateView();
            }

            // Alterna a câmera entre o braço robô e o pulso da garra
            if (keyboardState.IsKeyDown(Keys.NumPad7) && !keyboardStateAnt.IsKeyDown(Keys.NumPad7))
            {
                if (cameraCorrente != cameraPulsoGarra)
                    cameraCorrente = cameraPulsoGarra;
                else
                    cameraCorrente = cameraBracoRobo;

                UpdateView();
            }

            // Alterna a câmera entre o braço robô e o ponto da garra
            if (keyboardState.IsKeyDown(Keys.NumPad9) && !keyboardStateAnt.IsKeyDown(Keys.NumPad9))
            {
                if (cameraCorrente != cameraPontoGarra)
                    cameraCorrente = cameraPontoGarra;
                else
                    cameraCorrente = cameraBracoRobo;

                UpdateView();
            }

            // Distância da câmera ao alvo
            if (keyboardState.IsKeyDown(Keys.Add))
            {
                cameraCorrente.distanciaCamera -= 1;
                UpdateView();
            }
            else if (keyboardState.IsKeyDown(Keys.Subtract))
            {
                cameraCorrente.distanciaCamera += 1;
                UpdateView();
            }

            // Reseta as coordenadas da câmera para as posições padrão
            if (keyboardState.IsKeyDown(Keys.R))
            {
                cameraCorrente.ResetaPosicao();
                UpdateView();
            }

            // Elevação ou posição alvo da câmera
            if(keyboardState.IsKeyDown(Keys.Up))
            {
                // Sobe posição do alvo da câmera
                if (keyboardState.IsKeyDown(Keys.LeftShift) || keyboardState.IsKeyDown(Keys.RightShift))
                {
                    cameraCorrente.posicaoApontada += new Vector3(0.0f, 1.0f, 0.0f);
                }
                // Sobe elevação da câmera
                else if (cameraCorrente.elevacaoCamera > 0.01f)
                {
                    cameraCorrente.elevacaoCamera -= 1.0f;
                }
                else if (cameraCorrente.elevacaoCamera < 0.01f) // Para a imagem do braço robô não sumir
                {
                    cameraCorrente.elevacaoCamera = 0.01f;
                }
                UpdateView();                
            }
            else if(keyboardState.IsKeyDown(Keys.Down))
            {
                // Desce posição do alvo da câmera
                if (keyboardState.IsKeyDown(Keys.LeftShift) || keyboardState.IsKeyDown(Keys.RightShift))
                {
                    cameraCorrente.posicaoApontada -= new Vector3(0.0f, 1.0f, 0.0f);
                }
                else if (cameraCorrente.elevacaoCamera < 90.0f)
                {
                    cameraCorrente.elevacaoCamera += 1.0f;
                }
                else if (cameraCorrente.elevacaoCamera > 90.0f)
                {
                    cameraCorrente.elevacaoCamera = 90.0f;
                }

                UpdateView();
            }
            // Azimute da câmera
            if (keyboardState.IsKeyDown(Keys.Left))
            {
                cameraCorrente.azimuteCamera += 1;
                if (cameraCorrente.azimuteCamera > 360.0f)
                    cameraCorrente.azimuteCamera = cameraCorrente.azimuteCamera - 360.0f;

                UpdateView();
            }
            else if (keyboardState.IsKeyDown(Keys.Right))
            {
                cameraCorrente.azimuteCamera -= 1;
                if (cameraCorrente.azimuteCamera < 0.0f)
                    cameraCorrente.azimuteCamera = cameraCorrente.azimuteCamera + 360.0f;
                
                UpdateView();
            }

            ////// Comandos de teclado das juntas do braço robô //////

            // Base Giratória
            if (keyboardState.IsKeyDown(Keys.A))
            {                
                if (robo.motorBase.AnguloCorrente < robo.motorBase.anguloMax)
                    robo.motorBase.AnguloCorrente += 1;
                else
                    robo.motorBase.AnguloCorrente = robo.motorBase.anguloMax;
            }
            else if (keyboardState.IsKeyDown(Keys.Z))
            {                
                if (robo.motorBase.AnguloCorrente > robo.motorBase.anguloMin)
                    robo.motorBase.AnguloCorrente -= 1;
                else
                    robo.motorBase.AnguloCorrente = robo.motorBase.anguloMin;
            }
            // Braço L1
            if (keyboardState.IsKeyDown(Keys.S))
            {                
                if (robo.motor1.AnguloCorrente < robo.motor1.anguloMax)
                    robo.motor1.AnguloCorrente += 1;
                else
                    robo.motor1.AnguloCorrente = robo.motor1.anguloMax;
            }
            else if (keyboardState.IsKeyDown(Keys.X))
            {
                if (robo.motor1.AnguloCorrente > robo.motor1.anguloMin)
                    robo.motor1.AnguloCorrente -= 1;
                else
                    robo.motor1.AnguloCorrente = robo.motor1.anguloMin;
            }
            // Braço L2
            if (keyboardState.IsKeyDown(Keys.D))
            {
                if (robo.motor2.AnguloCorrente < robo.motor2.anguloMax)
                    robo.motor2.AnguloCorrente += 1;
                else
                    robo.motor2.AnguloCorrente = robo.motor2.anguloMax;
            }
            else if (keyboardState.IsKeyDown(Keys.C))
            {                
                if (robo.motor2.AnguloCorrente > robo.motor2.anguloMin)
                    robo.motor2.AnguloCorrente -= 1;
                else
                    robo.motor2.AnguloCorrente = robo.motor2.anguloMin;                
            }
            // Braço L3
            if (keyboardState.IsKeyDown(Keys.F))
            {
                if (robo.motor3.AnguloCorrente < robo.motor3.anguloMax)
                    robo.motor3.AnguloCorrente += 1;
                else
                    robo.motor3.AnguloCorrente = robo.motor3.anguloMax;
            }
            else if (keyboardState.IsKeyDown(Keys.V))
            {
                if (robo.motor3.AnguloCorrente > robo.motor3.anguloMin)
                    robo.motor3.AnguloCorrente -= 1;
                else
                    robo.motor3.AnguloCorrente = robo.motor3.anguloMin;
            }
            // Giro Garra
            if (keyboardState.IsKeyDown(Keys.G)) 
            {
                if (robo.motorGiroGarra.AnguloCorrente < robo.motorGiroGarra.anguloMax)
                    robo.motorGiroGarra.AnguloCorrente += 1;
                else
                    robo.motorGiroGarra.AnguloCorrente = robo.motorGiroGarra.anguloMax;
            }
            else if (keyboardState.IsKeyDown(Keys.B)) 
            {
                if (robo.motorGiroGarra.AnguloCorrente > robo.motorGiroGarra.anguloMin)
                    robo.motorGiroGarra.AnguloCorrente -= 1;
                else
                    robo.motorGiroGarra.AnguloCorrente = robo.motorGiroGarra.anguloMin;
            }
            // Garra (abertura e fechamento)
            if (keyboardState.IsKeyDown(Keys.H)) // fecha garra
            {
                if (robo.garra.servoDaGarra.AnguloCorrente < robo.garra.servoDaGarra.anguloMax)
                    robo.garra.servoDaGarra.AnguloCorrente += 1;
                else
                    robo.garra.servoDaGarra.AnguloCorrente = robo.garra.servoDaGarra.anguloMax;

                Console.WriteLine("ang eixo servo: " + robo.garra.servoDaGarra.eixo.anguloGiro.ToString());
                Console.WriteLine("ang engrenagem amort: " + robo.garra.eixoEngrenagemAmortecida.anguloGiro.ToString());
            }
            else if (keyboardState.IsKeyDown(Keys.N)) // abre garra
            {
                if (robo.garra.servoDaGarra.AnguloCorrente > robo.garra.servoDaGarra.anguloMin)
                    robo.garra.servoDaGarra.AnguloCorrente -= 1;
                else
                    robo.garra.servoDaGarra.AnguloCorrente = robo.garra.servoDaGarra.anguloMin;
                Console.WriteLine("ang eixo servo: " + robo.garra.servoDaGarra.eixo.anguloGiro.ToString());
                Console.WriteLine("ang engrenagem amort: " + robo.garra.eixoEngrenagemAmortecida.anguloGiro.ToString());
            }


            ///// Comandos para exibição/ocultação de objetos de cena /////

            // Habilita/desabilita os componentes a serem exibidos no braço robô, ou habilita tudo para exibição
            if (keyboardState.IsKeyDown(Keys.D0) && !keyboardStateAnt.IsKeyDown(Keys.D0))
            {
                if (posHabParte >=0 && posHabParte < habilitaParte.Length)
                {
                    for (int i = 0; i < habilitaParte.Length; i++)
                    {
                        habilitaParte[i] = false;
                    }
                    habilitaParte[posHabParte] = true;
                    posHabParte++;
                }
                else
                {
                    for (int i = 0; i < habilitaParte.Length; i++)
                    {
                        habilitaParte[i] = true;
                    }
                    posHabParte = 0;
                }
            }
            else if (keyboardState.IsKeyDown(Keys.D9) && !keyboardStateAnt.IsKeyDown(Keys.D9))
            {
                if (posHabParte >= 0 && posHabParte < habilitaParte.Length)
                {
                    for (int i = 0; i < habilitaParte.Length; i++)
                    {
                        habilitaParte[i] = false;
                    }
                    habilitaParte[posHabParte] = true;
                    posHabParte--;
                }
                else
                {
                    for (int i = 0; i < habilitaParte.Length; i++)
                    {
                        habilitaParte[i] = true;
                    }
                    posHabParte = habilitaParte.Length-1;
                }
            }
            else if (keyboardState.IsKeyDown(Keys.P))
            {
                if (keyboardState.IsKeyDown(Keys.LeftShift) || keyboardState.IsKeyDown(Keys.RightShift))
                {
                    for (int i = 0; i < frame.Length; i++)
                    {
                        frame[i].visivel = false;
                    }

                    referencialPontoGarra.visivel = false;
                    referencialPontoGarraDH.visivel = false;
                    planoBracoRobo.visivel = false;
                    planoGarra.visivel = false;
                    planoOrigemBaseFixaGarra.visivel = false;
                    conjuntoVetoresProjecao.visivel = false;
                }
                else
                {
                    for (int i = 0; i < habilitaParte.Length; i++)
                    {
                        habilitaParte[i] = true;
                    }
                    posHabParte = 0;
                }
            }

            // Referencial cartesiano da base fixa / Plano da origem da base fixa
            if (keyboardState.IsKeyDown(Keys.D1) && !keyboardStateAnt.IsKeyDown(Keys.D1))
            {
                if (keyboardState.IsKeyDown(Keys.LeftShift) || keyboardState.IsKeyDown(Keys.RightShift))
                {
                    // Exibe/oculta o plano do braço robô que passa pela origem da base fixa
                    planoBracoRobo.visivel = !planoBracoRobo.visivel;
                }
                else
                {
                    // Exibe/oculta o referencial cartesiano da base fixa
                    referencialBaseFixa.visivel = !referencialBaseFixa.visivel;
                }
            }

            // Referencial cartesiano da base giratória / Plano do ponto da garra
            if (keyboardState.IsKeyDown(Keys.D2) && !keyboardStateAnt.IsKeyDown(Keys.D2))
            {
                if (keyboardState.IsKeyDown(Keys.LeftShift) || keyboardState.IsKeyDown(Keys.RightShift))
                {
                    // Exibe/oculta o plano do braço robô que passa pelo ponto da garra
                    planoGarra.visivel = !planoGarra.visivel;
                }
                else
                {
                    // Exibe/oculta o referencial cartesiano da base giratória
                    frame[1].visivel = !frame[1].visivel;
                }
            }

            // Referencial cartesiano do segmento L1 / Plano origem da base fixa e ponto da garra
            if (keyboardState.IsKeyDown(Keys.D3) && !keyboardStateAnt.IsKeyDown(Keys.D3))
            {
                if (keyboardState.IsKeyDown(Keys.LeftShift) || keyboardState.IsKeyDown(Keys.RightShift))
                {
                    // Exibe/oculta o plano do braço robô que passa pela origem da base fixa e pelo ponto da garra
                    planoOrigemBaseFixaGarra.visivel = !planoOrigemBaseFixaGarra.visivel;
                }
                else
                {
                    // Exibe/oculta o referencial cartesiano do segmento L1
                    frame[2].visivel = !frame[2].visivel;
                }
            }

            // Exibe/oculta o referencial cartesiano do segmento L2
            if (keyboardState.IsKeyDown(Keys.D4) && !keyboardStateAnt.IsKeyDown(Keys.D4))
            {
                frame[3].visivel = !frame[3].visivel;
            }

            // Exibe/oculta o referencial cartesiano do segmento L3
            if (keyboardState.IsKeyDown(Keys.D5) && !keyboardStateAnt.IsKeyDown(Keys.D5))
            {
                frame[4].visivel = !frame[4].visivel;
            }

            // Exibe/oculta o referencial cartesiano do pulso da garra
            if (keyboardState.IsKeyDown(Keys.D6) && !keyboardStateAnt.IsKeyDown(Keys.D6))
            {
                frame[5].visivel = !frame[5].visivel;
            }
            
            if (keyboardState.IsKeyDown(Keys.D7) && !keyboardStateAnt.IsKeyDown(Keys.D7))
            {
                if (keyboardState.IsKeyDown(Keys.LeftShift) || keyboardState.IsKeyDown(Keys.RightShift))
                {
                    // Exibe/oculta o referencial cartesiano do ponto da garra (ponto exato)
                    referencialPontoGarra.visivel = !referencialPontoGarra.visivel;
                }
                else
                {
                    // Exibe/oculta o referencial cartesiano do ponto da garra calculado pelos parâmetros DH
                    referencialPontoGarraDH.visivel = !referencialPontoGarraDH.visivel;
                }
            }

            // Exibe/oculta o conjunto de vetores de projeção
            if (keyboardState.IsKeyDown(Keys.D8) && !keyboardStateAnt.IsKeyDown(Keys.D8))
            {
                conjuntoVetoresProjecao.visivel = !conjuntoVetoresProjecao.visivel;
            }

            // Gira o vetor Zt para perto/longe do vetor Ztl
            if (keyboardState.IsKeyDown(Keys.NumPad4))
            {
                if (conjuntoVetoresProjecao.visivel)
                {
                    if (conjuntoVetoresProjecao.anguloTetaZtRot > -180.0f)
                        conjuntoVetoresProjecao.anguloTetaZtRot -= 1.0f;
                    else
                        conjuntoVetoresProjecao.anguloTetaZtRot = 180.0f;
                }
            }
            else if (keyboardState.IsKeyDown(Keys.NumPad6))
            {
                if (conjuntoVetoresProjecao.visivel)
                {
                    if (conjuntoVetoresProjecao.anguloTetaZtRot < 180.0f)
                        conjuntoVetoresProjecao.anguloTetaZtRot += 1.0f;
                    else
                        conjuntoVetoresProjecao.anguloTetaZtRot = -180.0f;
                }
            }

            // Modo posição alvo ativado/desativado para o conjunto de vetores de projeção
            if (keyboardState.IsKeyDown(Keys.NumPad5) && !keyboardStateAnt.IsKeyDown(Keys.NumPad5))
            {
                if (conjuntoVetoresProjecao.visivel)
                {
                    conjuntoVetoresProjecao.determinadoPorPosicaoAlvo = !conjuntoVetoresProjecao.determinadoPorPosicaoAlvo;

                    if (conjuntoVetoresProjecao.determinadoPorPosicaoAlvo)
                    {
                        Console.WriteLine("Modo posição alvo do conjunto de vetores de projeção ATIVADO.");
                        Console.WriteLine("Digite NumPad 0 para digitar a posição alvo.");
                        
                        conjuntoVetoresProjecao.AtualizarMatrizPosicaoAlvo();
                    }
                    else
                        Console.WriteLine("Modo posição alvo do conjunto de vetores de projeção DESATIVADO.");
                }
            }

            // Habilita entrada de coodenadas da posição alvo do conjunto de vetores de projeção
            if (keyboardState.IsKeyDown(Keys.NumPad0) && !keyboardStateAnt.IsKeyDown(Keys.NumPad0))
            {
                if (conjuntoVetoresProjecao.visivel && conjuntoVetoresProjecao.determinadoPorPosicaoAlvo)
                {
                    Console.WriteLine("Digite as coodenadas x, y, z, Rx, Ry, Rz alvo da garra com");
                    Console.WriteLine("ponto decimal e separadas por vírgula, e digite ENTER ao");
                    Console.WriteLine("final, ou simplesmente ENTER para sair:");
                    coordenadasXYZGamaBetaAlfa = Console.ReadLine();
                    coordenadasXYZGamaBetaAlfa.Trim();

                    if (conjuntoVetoresProjecao.expressaoRegularPosAlvo.IsMatch(coordenadasXYZGamaBetaAlfa))
                    {
                        String strNum = String.Empty;
                        float[] valores = new float[6];

                        coordenadasXYZGamaBetaAlfa += "$"; // Para indicar fim de entrada para o laço "for"
                        int idx = 0;
                        for (int i = 0; i < coordenadasXYZGamaBetaAlfa.Length; i++)
                        {
                            if (coordenadasXYZGamaBetaAlfa[i] != ',' && coordenadasXYZGamaBetaAlfa[i] != '$')
                                strNum += coordenadasXYZGamaBetaAlfa[i];
                            else
                            {
                                // Reconhece e converte para inteiro
                                strNum.Trim();
                                strNum = strNum.Replace('.', ',');
                                valores[idx++] = float.Parse(strNum);
                                strNum = String.Empty;
                            }
                        }

                        conjuntoVetoresProjecao.xyzAlvo = new Vector3(valores[0], valores[1], valores[2]);
                        conjuntoVetoresProjecao.gamaAlvo = valores[3];
                        conjuntoVetoresProjecao.betaAlvo = valores[4];
                        conjuntoVetoresProjecao.alfaAlvo = valores[5];

                        Console.WriteLine("Posição alvo reconhecida:");
                        Console.WriteLine("X = {0}", conjuntoVetoresProjecao.xyzAlvo.X);
                        Console.WriteLine("Y = {0}", conjuntoVetoresProjecao.xyzAlvo.Y);
                        Console.WriteLine("Z = {0}", conjuntoVetoresProjecao.xyzAlvo.Z);
                        Console.WriteLine("gama = {0}", conjuntoVetoresProjecao.gamaAlvo);
                        Console.WriteLine("beta = {0}", conjuntoVetoresProjecao.betaAlvo);
                        Console.WriteLine("alfa = {0}", conjuntoVetoresProjecao.alfaAlvo);

                        conjuntoVetoresProjecao.AtualizarMatrizPosicaoAlvo();
                    }
                    else if (coordenadasXYZGamaBetaAlfa.Length > 0)
                        Console.WriteLine("Posição alvo não reconhecida.");
                    else
                        Console.WriteLine("Nenhuma posição alvo especificada.");

                }
            }

            // Mostra no console a coordenada da posição desejada da garra a partir do vetor Zt
            if (keyboardState.IsKeyDown(Keys.NumPad8) && !keyboardStateAnt.IsKeyDown(Keys.NumPad8))
            {
                if (conjuntoVetoresProjecao.visivel)
                {
                    Vector3 translacaoZt;
                    Vector3 posAlvoDesejada;

                    Matrix R;

                    double r11;
                    double r21;
                    double r31;
                    
                    double r12;
                    double r22;
                    double r32;
                    
                    //double r13;
                    //double r23;
                    double r33;

                    Vector3 X;
                    Vector3 Y;
                    Vector3 Z;

                    if (conjuntoVetoresProjecao.determinadoPorPosicaoAlvo)
                    {
                        // Este processo (e o cálculo dos ângulos), embora desnecessário, é mais para 
                        // validar se matrizPosZtGarra está sendo corretamente calculada.
                        // Uma alternativa seria simplesmente exibir os valores dos campos
                        // xyzAlvo, gamaAlvo, betaAlvo, alfaAlvo e anguloTetaZtProj
                        translacaoZt = conjuntoVetoresProjecao.matrizPosZtGarra.Translation;
                        posAlvoDesejada = translacaoZt;

                        R = ExtraiMatrizRotacao(conjuntoVetoresProjecao.matrizPosZtGarra);                        

                        X = new Vector3(R.M11, R.M12, R.M13);
                        Y = new Vector3(R.M21, R.M22, R.M23);
                        Z = new Vector3(R.M31, R.M32, R.M33);

                        r11 = Math.Round(X.X, 4);
                        r21 = Math.Round(X.Y, 4);
                        r31 = Math.Round(X.Z, 4);

                        r12 = Math.Round(Y.X, 4);
                        r22 = Math.Round(Y.Y, 4);
                        r32 = Math.Round(Y.Z, 4);

                        //r13 = Z.X;
                        //r23 = Z.Y;
                        r33 = Math.Round(Z.Z, 4);
                    }
                    else
                    {
                        translacaoZt = conjuntoVetoresProjecao.matrizPosZtGarra.Translation;
                        posAlvoDesejada = new Vector3((float)Math.Round(-translacaoZt.Z, 1),
                                                      (float)Math.Round(-translacaoZt.X, 1),
                                                      (float)Math.Round(translacaoZt.Y, 1));

                        R = ExtraiMatrizRotacao(conjuntoVetoresProjecao.matrizPosZtGarra);

                        X = new Vector3(R.M31, R.M32, R.M33);
                        Y = new Vector3(R.M11, R.M12, R.M13);
                        Z = new Vector3(R.M21, R.M22, R.M23);

                        r11 = Math.Round(X.Z, 4);
                        r21 = Math.Round(X.X, 4);
                        r31 = Math.Round(-X.Y, 4);

                        r12 = Math.Round(Y.Z, 4);
                        r22 = Math.Round(Y.X, 4);
                        r32 = Math.Round(Y.Y, 4);

                        //r13 = -Z.Z;
                        //r23 = Z.X;
                        r33 = Math.Round(Z.Y, 4);
                    }

                    double beta = Math.Atan2(-r31, Math.Sqrt(Math.Pow(r11, 2) + Math.Pow(r21, 2)));
                    double alfa, gama;

                    double betaGraus = beta * 180 / Math.PI;
                    betaGraus = Math.Round(betaGraus, 2);
                    betaGraus = Math.Round(betaGraus, 1);

                    if (betaGraus == 90.0)
                    {
                        alfa = 0;
                        gama = Math.Atan2(r12, r22);
                    }
                    else if (betaGraus == -90.0)
                    {
                        alfa = 0;
                        gama = -Math.Atan2(r12, r22);
                    }
                    else
                    {
                        double cbeta = Math.Cos(beta);

                        alfa = Math.Atan2(r21 / cbeta, r11 / cbeta);
                        gama = Math.Atan2(r32 / cbeta, r33 / cbeta);
                    }

                    alfa = Math.Round(alfa, 8);
                    gama = Math.Round(gama, 8);

                    alfa = MathHelper.ToDegrees((float)alfa);
                    beta = MathHelper.ToDegrees((float)beta);
                    gama = MathHelper.ToDegrees((float)gama);

                    alfa = Math.Round(alfa, 1);
                    beta = Math.Round(beta, 1);
                    gama = Math.Round(gama, 1);

                    Console.WriteLine("Posição desejada da garra:");
                    Console.WriteLine("X = {0} cm", posAlvoDesejada.X);
                    Console.WriteLine("Y = {0} cm", posAlvoDesejada.Y);
                    Console.WriteLine("Z = {0} cm", posAlvoDesejada.Z);

                    Console.WriteLine("Rx = {0}°", gama);
                    Console.WriteLine("Ry = {0}°", beta);
                    Console.WriteLine("Rz = {0}°", alfa);

                    Console.WriteLine("teta de projeção de Zt: {0}", conjuntoVetoresProjecao.anguloTetaZtProj);
                }
            }

            // Alterna entre os referenciais de Denavit-Hatenberg precisos e os adaptados
            if (keyboardState.IsKeyDown(Keys.Tab) && !keyboardStateAnt.IsKeyDown(Keys.Tab))
            {
                parametrosDHExatos = !parametrosDHExatos;

                AlternaParametrosDH();                
            }

            // Exibe o índice da ajuda
            if (keyboardState.IsKeyDown(Keys.F1) && !keyboardStateAnt.IsKeyDown(Keys.F1))
            {
                IndiceDaAjuda();
            }
            // Ajuda da câmera e suas coordenadas
            else if (keyboardState.IsKeyDown(Keys.F2) && !keyboardStateAnt.IsKeyDown(Keys.F2))
            {
                AjudaCamera();
            }
            // Ajuda dos comandos das juntas
            else if (keyboardState.IsKeyDown(Keys.F3) && !keyboardStateAnt.IsKeyDown(Keys.F3))
            {
                AjudaComandosDasJuntas();
            }
            // Ajuda da exibição/ocultação de partes do braço robô
            else if (keyboardState.IsKeyDown(Keys.F4) && !keyboardStateAnt.IsKeyDown(Keys.F4))
            {
                AjudaPartesBracoRobo();
            }
            // Ajuda da exibição/ocultação dos referenciais cartesianos (frames)
            else if (keyboardState.IsKeyDown(Keys.F5) && !keyboardStateAnt.IsKeyDown(Keys.F5))
            {
                AjudaReferenciaisCartesianos();
            }
            // Ajuda da exibição/ocultação dos planos verticais
            else if (keyboardState.IsKeyDown(Keys.F6) && !keyboardStateAnt.IsKeyDown(Keys.F6))
            {
                AjudaPlanosVerticais();
            }
            // Ajuda dos comandos para o conjunto de vetores de projeção
            else if (keyboardState.IsKeyDown(Keys.F7) && !keyboardStateAnt.IsKeyDown(Keys.F7))
            {
                AjudaConjuntoDeVetoresDeProjecao();
            }
            // Sobre...
            else if (keyboardState.IsKeyDown(Keys.F8) && !keyboardStateAnt.IsKeyDown(Keys.F8))
            {
                Sobre();
            }
        }

        /// <summary>
        /// Atualiza posicionamento da câmera corrente
        /// </summary>
        void UpdateView()
        {            
            cameraCorrente.UpdatePosicao();
            view = cameraCorrente.MatrizView();
        }

        /// <summary>
        /// Permite a simulação (jogo) rodar lógica tal como atualizar o mundo (cenário),
        /// checar colisões, obtendo entradas e tocando audio.        
        /// </summary>
        /// <param name="gameTime">Provê um retrato dos valores de tempo.</param>
        protected override void Update(GameTime gameTime)
        {
            UpdateKeyboard();

            robo.Update(gameTime);

            base.Update(gameTime);
        }

        /// <summary>
        /// Método para ajustar o tamanho do eixo X1, quando o frame 1 for visível
        /// </summary>
        private void AjustaEixoX1()
        {
            if (robo.motor1.AnguloCorrenteDH >= 15.0f)
            {
                frame[1].TamanhoEixoX = frame[1].tamanhoEixoXOriginal;
                frame[1].posicaoLetraX.Z = frame[1].posicaoLetraXOriginal.Z;                
            }
            else
            {
                frame[1].TamanhoEixoX = 2.5f;
                frame[1].posicaoLetraX.Z = -4.0f;
            }
        }

        /// <summary>
        /// Atualiza a posição apontada pela câmera do pulso da garra
        /// </summary>
        /// <param name="posicaoPulsoGarra">posição do pulso da garra</param>
        private void AtualizaPosicaoApontadaCameraPulsoGarra(Vector3 posicaoPulsoGarra)
        {
            cameraPulsoGarra.posicaoApontada = posicaoPulsoGarra;
            cameraPulsoGarra.posicaoApontadaInicial = posicaoPulsoGarra;
        }

        /// <summary>
        /// Atualiza a posição apontada pela câmera do ponto da garra
        /// </summary>
        /// <param name="posicaoPontoGarra">posição do ponto da garra</param>
        private void AtualizaPosicaoApontadaCameraPontoGarra(Vector3 posicaoPontoGarra)
        {
            cameraPontoGarra.posicaoApontada = posicaoPontoGarra;
            cameraPontoGarra.posicaoApontadaInicial = posicaoPontoGarra;
        }


        /// <summary>
        /// Este método é chamado quando a simulação (jogo) deve se desenhar
        /// </summary>
        /// <param name="gameTime">Provê um retrato dos valores de tempo.</param>
        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.DarkOliveGreen);

            Matrix matrizAux;
            List<Matrix> listaMatrizes;

            float[] teta = new float[6];

            for(int i = 1; i < 6; i++)
            {
                teta[i] = MathHelper.ToRadians(robo.placaMiniMaestro24.canais[i-1].servo.AnguloCorrenteDH);
            }

            Matrix[] RzTeta = new Matrix[6];

            for(int i = 1; i < 6; i++)
            {
                RzTeta[i] = Matrix.CreateRotationY(teta[i]);
            }

            // Base Fixa
            Matrix matrizBaseFixa = Matrix.Identity;
            matrizAux = matrizBaseFixa;
            if (habilitaParte[0])
            {
                DrawBaseFixaComLEDS(robo.bufferLeds, robo.baseFixa.modelo, matrizAux, view, projection);                
            }

            // Referencial cartesiano da base fixa
            if (referencialBaseFixa.visivel)
            {
                DrawReferencialCartesiano(referencialBaseFixa, matrizAux, view, projection);
            }
            // Servo base
            listaMatrizes = robo.motorBase.Matrizes(matrizAux);
            if (habilitaParte[1])
                DrawModelMultiMesh(robo.motorBase.modelo, world, listaMatrizes, view, projection);

            // base giratória
            matrizAux = robo.baseGiratoria.MatrizRotacaoX()
                      * robo.baseGiratoria.MatrizRotacaoY()
                      * robo.baseGiratoria.MatrizRotacaoZ()
                      * (robo.baseGiratoria.MatrizTranslacao() * listaMatrizes[listaMatrizes.Count - 1]);
            if (habilitaParte[2])
                DrawModelMonoMesh(robo.baseGiratoria.modelo, matrizAux, view, projection);

            // Referencial cartesiano da base giratória
            Matrix T01 = RzTeta[1] * Dzd[1] * RxAlfa[0] * Dxa[0];
            if (frame[1].visivel)
            {
                AjustaEixoX1();
                DrawReferencialCartesiano(frame[1], T01, view, projection);
            }

            // Plano que corta o braço robô ao meio
            if (planoBracoRobo.visivel)
            {
                Matrix matrizPlanoBracoRobo = robo.motorBase.eixo.MatrizRotacaoX()
                                            * robo.motorBase.eixo.MatrizRotacaoY()
                                            * robo.motorBase.eixo.MatrizRotacaoZ()
                                            * matrizBaseFixa;
                DrawPlano(planoBracoRobo, matrizPlanoBracoRobo, view, projection);
            }

            // Plano que corta a garra ao meio
            if (planoGarra.visivel)
            {
                Matrix matrizPlanoGarra = Matrix.CreateTranslation(-1.3161f, 0.0f, 0.0f)
                                        * robo.motorBase.eixo.MatrizRotacaoX()
                                        * robo.motorBase.eixo.MatrizRotacaoY()
                                        * robo.motorBase.eixo.MatrizRotacaoZ()                                        
                                        * matrizBaseFixa;
                DrawPlano(planoGarra, matrizPlanoGarra, view, projection);
            }

            // Servo M1
            listaMatrizes = robo.motor1.Matrizes(matrizAux);
            if (habilitaParte[3])
                DrawModelMultiMesh(robo.motor1.modelo, world, listaMatrizes, view, projection);

            // Segmento 1
            matrizAux = robo.bracoL1.MatrizRotacaoX()
                      * robo.bracoL1.MatrizRotacaoY()
                      * robo.bracoL1.MatrizRotacaoZ()
                      * (robo.bracoL1.MatrizTranslacao() * listaMatrizes[listaMatrizes.Count - 1]);
            if (habilitaParte[4])
                DrawModelMonoMesh(robo.bracoL1.modelo, matrizAux, view, projection);

            // Referencial cartesiano do segmento 1
            Matrix T12 = Dzd[2] * RzTeta[2] * RxAlfa[1] * Dxa[1];
            Matrix T02 = T12 * T01;
            if(frame[2].visivel)
                DrawReferencialCartesiano(frame[2], T02, view, projection);

            // Servo M2
            listaMatrizes = robo.motor2.Matrizes(matrizAux);
            if (habilitaParte[5])
                DrawModelMultiMesh(robo.motor2.modelo, world, listaMatrizes, view, projection);

            // Segmento 2
            matrizAux = robo.bracoL2.MatrizRotacaoX()
                      * robo.bracoL2.MatrizRotacaoY()
                      * robo.bracoL2.MatrizRotacaoZ()
                      * (robo.bracoL2.MatrizTranslacao() * listaMatrizes[listaMatrizes.Count - 1]);
            if (habilitaParte[6])
                DrawModelMonoMesh(robo.bracoL2.modelo, matrizAux, view, projection);

            // Referencial cartesiano do segmento 2
            Matrix T23 = Dzd[3] * RzTeta[3] * RxAlfa[2] * Dxa[2];
            Matrix T03 = T23 * T02;
            if(frame[3].visivel)
                DrawReferencialCartesiano(frame[3], T03, view, projection);

            // Servo M3
            listaMatrizes = robo.motor3.Matrizes(matrizAux);
            if (habilitaParte[7])
                DrawModelMultiMesh(robo.motor3.modelo, world, listaMatrizes, view, projection);

            // Segmento 3
            matrizAux = robo.bracoL3.MatrizRotacaoX()
                      * robo.bracoL3.MatrizRotacaoY()
                      * robo.bracoL3.MatrizRotacaoZ()
                      * (robo.bracoL3.MatrizTranslacao() * listaMatrizes[listaMatrizes.Count - 1]);
            if (habilitaParte[8])
                DrawModelMonoMesh(robo.bracoL3.modelo, matrizAux, view, projection);

            // Referencial cartesiano do segmento 3
            Matrix T34 = Dzd[4] * RzTeta[4] * RxAlfa[3] * Dxa[3];
            Matrix T04 = T34 * T03;
            if(frame[4].visivel)
                DrawReferencialCartesiano(frame[4], T04, view, projection);

            // Servo do pulso da garra
            listaMatrizes = robo.motorGiroGarra.Matrizes(matrizAux);
            if (habilitaParte[9])
                DrawModelMultiMesh(robo.motorGiroGarra.modelo, world, listaMatrizes, view, projection);

            // Garra
            listaMatrizes = robo.garra.Matrizes(listaMatrizes[listaMatrizes.Count - 1]);
            if (habilitaParte[10])
                DrawModelMultiMesh(robo.garra.modelo, world, listaMatrizes, view, projection);

            // Referencial cartesiano do pulso da garra
            Matrix T45 = Dzd[5] * RzTeta[5] * RxAlfa[4] * Dxa[4];
            Matrix T05 = T45 * T04;
            if(frame[5].visivel)
                DrawReferencialCartesiano(frame[5], T05, view, projection);

            if (cameraCorrente == cameraPulsoGarra)
            {
                Vector3 pontoPulsoGarra = T05.Translation;
                AtualizaPosicaoApontadaCameraPulsoGarra(pontoPulsoGarra);
                UpdateView();
            }            

            Matrix matrizBaseGarra = listaMatrizes[0];

            // Servo da garra
            listaMatrizes = robo.garra.servoDaGarra.Matrizes(matrizBaseGarra); // Base da garra como referencial
            if (habilitaParte[11])
                DrawModelMultiMesh(robo.garra.servoDaGarra.modelo, world, listaMatrizes, view, projection);

            // Referencial cartesiano do ponto da garra (ponto exato)
            Matrix matrizPontoGarra = Matrix.CreateRotationY(MathHelper.ToRadians(180.0f)) * Matrix.CreateTranslation(0.0f, 7.5f, 0.0f) * matrizBaseGarra;
            if (referencialPontoGarra.visivel)
            {
                DrawReferencialCartesiano(referencialPontoGarra, matrizPontoGarra, view, projection);
            }

            // Referencial cartesiano do ponto da garra (calculado via parâmetros DH)
            float tamanhoL3EGarra;
            if (parametrosDHExatos)
                tamanhoL3EGarra = 7.5f - robo.garra.posReferencial.Y - 0.21258f;
            else
                tamanhoL3EGarra = robo.bracoL3.comprimento + 7.5f - robo.garra.posReferencial.Y - 0.21258f;

            Matrix TGarraDH = Matrix.CreateTranslation(0.0f, tamanhoL3EGarra, 0.0f) * T05;
            if (referencialPontoGarraDH.visivel)
                DrawReferencialCartesiano(referencialPontoGarraDH, TGarraDH, view, projection);

            Vector3 pontoOrigemBaseFixa = matrizBaseFixa.Translation;
            Vector3 pontoGarra = matrizPontoGarra.Translation;
            Vector3 vetorPontoGarra = pontoGarra - pontoOrigemBaseFixa;

            float anguloPlanoObliquo = (float)Math.Atan2(vetorPontoGarra.X, vetorPontoGarra.Z) - MathHelper.ToRadians(robo.motorBase.AnguloCorrente) - MathHelper.Pi;

            // Conjunto de vetores de projeção
            if (conjuntoVetoresProjecao.visivel)
            {
                if (conjuntoVetoresProjecao.determinadoPorPosicaoAlvo)
                {
                    conjuntoVetoresProjecao.world = conjuntoVetoresProjecao.matrizPosicaoAlvo;
                }
                else
                {                    
                    if (parametrosDHExatos)
                    {
                        Matrix matrizVetorMObliquo = Matrix.CreateRotationY(anguloPlanoObliquo)
                                                   * robo.motorBase.eixo.MatrizRotacaoX()
                                                   * robo.motorBase.eixo.MatrizRotacaoY()
                                                   * robo.motorBase.eixo.MatrizRotacaoZ()
                                                   * matrizBaseFixa;

                        Matrix matrizPosConjVetores = (matrizVetorMObliquo * Dzd[1]) * (RxAlfa[0] * Dxa[0]);
                        matrizPosConjVetores = T12 * matrizPosConjVetores;
                        matrizPosConjVetores = T23 * matrizPosConjVetores;
                        matrizPosConjVetores = T34 * matrizPosConjVetores;
                        matrizPosConjVetores = (Dzd[5] * RxAlfa[4] * Dxa[4]) * matrizPosConjVetores;
                        
                        conjuntoVetoresProjecao.world = Matrix.CreateTranslation(0.0f, tamanhoL3EGarra, 0.0f) * matrizPosConjVetores;

                        float teta1aux = MathHelper.ToRadians(robo.motorBase.AnguloCorrenteDH);

                        Matrix matrizPosPlanConjVetores = Matrix.Identity;

                        for (int i = 0; i < 5; i++)
                        {
                            matrizPosPlanConjVetores = (RzTeta[i + 1] * DzdPlan[i + 1]) * (RxAlfaPlan[i] * DxaPlan[i]) * matrizPosPlanConjVetores;
                        }

                        matrizPosPlanConjVetores = Matrix.CreateTranslation(0.0f, tamanhoL3EGarra, 0.0f) * matrizPosPlanConjVetores;

                        float xAux = -matrizPosPlanConjVetores.Translation.Z;
                        float yAux = -matrizPosPlanConjVetores.Translation.X;

                        if (anguloPlanoObliquo < 0 && 
                            ((teta1aux < 0 && yAux > 0) ||
                             (teta1aux > 0 && yAux < 0) ||
                             (teta1aux == 0 && xAux < 0)))
                        {
                            Matrix R = ExtraiMatrizRotacao(conjuntoVetoresProjecao.world);

                            Vector3 X = new Vector3(R.M31, R.M32, R.M33);
                            Vector3 Y = new Vector3(R.M11, R.M12, R.M13);
                            Vector3 Z = new Vector3(R.M21, R.M22, R.M23);

                            double r11 = Math.Round(X.Z, 4);
                            double r21 = Math.Round(X.X, 4);
                            double r31 = Math.Round(X.Y, 4);

                            double r12 = Math.Round(Y.Z, 4);
                            double r22 = Math.Round(Y.X, 4);
                            double r32 = Math.Round(Y.Y, 4);

                            //double r13;
                            //double r23;
                            double r33 = Math.Round(Z.Y, 4);
                            
                            double beta = Math.Atan2(-r31, Math.Sqrt(Math.Pow(r11, 2) + Math.Pow(r21, 2)));
                            double alfa, gama;

                            double betaGraus = beta * 180 / Math.PI;
                            betaGraus = Math.Round(betaGraus, 2);
                            betaGraus = Math.Round(betaGraus, 1);

                            if (betaGraus == 90.0)
                            {
                                alfa = 0;
                                gama = Math.Atan2(r12, r22);
                            }
                            else if (betaGraus == -90.0)
                            {
                                alfa = 0;
                                gama = -Math.Atan2(r12, r22);
                            }
                            else
                            {
                                double cbetal = Math.Cos(beta);

                                alfa = Math.Atan2(r21 / cbetal, r11 / cbetal);
                                gama = Math.Atan2(r32 / cbetal, r33 / cbetal);
                            }

                            beta = -beta;
                            
                            conjuntoVetoresProjecao.world = Matrix.CreateRotationZ((float)gama)
                                                          * Matrix.CreateRotationX((float)beta)
                                                          * Matrix.CreateRotationY((float)alfa);
                        }

                        conjuntoVetoresProjecao.world.Translation = pontoGarra;
                    }
                    else
                    {
                        conjuntoVetoresProjecao.world = (Dzd[5] * RxAlfa[4] * Dxa[4]) * T04;
                        conjuntoVetoresProjecao.world = Matrix.CreateTranslation(0.0f, tamanhoL3EGarra, 0.0f) * conjuntoVetoresProjecao.world;
                    }

                    float teta1 = robo.motorBase.AnguloCorrenteDH;

                    Vector3 posicaoConjProj = conjuntoVetoresProjecao.world.Translation;

                    float x = -posicaoConjProj.Z;
                    float y = -posicaoConjProj.X;
                    
                    conjuntoVetoresProjecao.inverteVetoresMeK = (parametrosDHExatos)|| ((teta1 < 0 && y < 0) || (teta1 > 0 && y > 0) || (teta1 == 0 && x >= 0));
                }

                DrawConjuntoVetoresProjecao(conjuntoVetoresProjecao, conjuntoVetoresProjecao.world, view, projection);
            }

            if (cameraCorrente == cameraPontoGarra)
            {
                Vector3 pontoGarraDH = TGarraDH.Translation;
                AtualizaPosicaoApontadaCameraPontoGarra(pontoGarraDH);
                UpdateView();
            }

            // Plano que passa pela origem da base fixa e pelo ponto da garra
            if (planoOrigemBaseFixaGarra.visivel)
            {
                Matrix matrizOrigemBaseFixaGarra = Matrix.CreateRotationY(anguloPlanoObliquo)
                                                 * robo.motorBase.eixo.MatrizRotacaoX()
                                                 * robo.motorBase.eixo.MatrizRotacaoY()
                                                 * robo.motorBase.eixo.MatrizRotacaoZ()
                                                 * matrizBaseFixa;
                DrawPlano(planoOrigemBaseFixaGarra, matrizOrigemBaseFixaGarra, view, projection);
            }

            // Objeto segurado pela garra
            //if (robo.garra.objeto != null)
            //{
            //    matrizAux = robo.garra.objeto.MatrizRotacaoX() * robo.garra.objeto.MatrizRotacaoY() * robo.garra.objeto.MatrizRotacaoZ() * (robo.garra.objeto.MatrizTranslacao() * matrizBaseGarra);
            //    if (habilitaParte[12])
            //        DrawModelMonoMesh(robo.garra.objeto.modelo, matrizAux, view, projection);
            //}

            base.Draw(gameTime);
        }

        /// <summary>
        /// Desenha um único mesh de um modelo 3D
        /// </summary>
        /// <param name="mesh">mesh a ser desenhado</param>
        /// <param name="world">matriz do referencial do objeto</param>
        /// <param name="view">matriz da câmera</param>
        /// <param name="projection">matriz da projeção</param>
        /// <param name="alpha">ajusta a transparência entre 0 (totalmente transparente) e 1 (totalmente opaco (default))</param>
        private void DrawMesh(ModelMesh mesh, Matrix world, Matrix view, Matrix projection, float alpha = 1.0f)
        {
            foreach (BasicEffect effect in mesh.Effects)
            {
                effect.EnableDefaultLighting();
                
                effect.Alpha = alpha;

                //effect.PreferPerPixelLighting = true;

                //effect.LightingEnabled = true; // turn on the lighting subsystem.

                effect.AmbientLightColor = new Vector3(0.1f, 0.1f, 0.1f);

                effect.World = world;
                effect.View = view;
                effect.Projection = projection;
            }

            mesh.Draw();
        }

        /// <summary>
        /// Desenha os modelos 3D do robô que possuam apenas um mesh ou que possua vários meshs, mas que a posição de
        /// todos eles sejam determinados pela mesma matriz.
        /// </summary>
        /// <param name="model">modelo 3D a ser desenhado</param>
        /// <param name="world">matriz do referencial do objeto</param>
        /// <param name="view">matriz da câmera</param>
        /// <param name="projection">matriz da projeção</param>
        private void DrawModelMonoMesh(Model model, Matrix world, Matrix view, Matrix projection)
        {
            foreach (ModelMesh mesh in model.Meshes)
            {
                DrawMesh(mesh, world, view, projection);
            }
        }

        /// <summary>
        /// Desenha os modelos 3D do robô que possuam vários meshs controlados cada um por suas respectivas matrizes.
        /// </summary>
        /// <param name="model">modelo 3D a ser desenhado</param>
        /// <param name="objectWorldMatrix">matriz do referencial do objeto</param>
        /// <param name="meshWorldMatrices">Lista de matrizesGarra das partes do modelo</param>
        /// <param name="view">matriz da câmera</param>
        /// <param name="projection">matriz da projeção</param>
        private void DrawModelMultiMesh(Model model, Matrix objectWorldMatrix, List<Matrix> meshWorldMatrices, Matrix view, Matrix projection)
        {
            for (int index = 0; index < model.Meshes.Count; index++)
            {
                ModelMesh mesh = model.Meshes[index];
                Matrix world = mesh.ParentBone.Transform * meshWorldMatrices[index] * objectWorldMatrix;

                DrawMesh(mesh, world, view, projection);
            }
        }

        /// <summary>
        /// Desenha a base fixa com a placa Ready for PIC e a placa Easyled
        /// </summary>
        /// <param name="ledAceso">Vetor que contém os estados de cada led da placa Easyled</param>
        /// <param name="model">modelo 3D a ser desenhado</param>
        /// <param name="world">matriz do referencial do objeto</param>
        /// <param name="view">matriz da câmera</param>
        /// <param name="projection">matriz da projeção</param>
        private void DrawBaseFixaComLEDS(bool[] ledAceso, Model model, Matrix world, Matrix view, Matrix projection)
        {
            // Desenha a base fixa mais as placas Ready for PIC e a Easyled (com os leds apagados)
            ModelMesh mesh = model.Meshes[0];
            DrawMesh(mesh, world, view, projection);

            // Desenha as luzes dos leds que estiverem acesos
            for (int i = 0; i < 8; i++)
            {
                if (ledAceso[i])
                {
                    mesh = model.Meshes[i + 1];
                    DrawMesh(mesh, world, view, projection);
                }
            }
        }
        /// <summary>
        /// Desenha um referencial cartesiano
        /// </summary>
        /// <param name="referencial">Objeto que representa o referencial cartesiano</param>
        /// <param name="world">matriz do referencial do objeto</param>
        /// <param name="view">matriz da câmera</param>
        /// <param name="projection">matriz da projeção</param>
        private void DrawReferencialCartesiano(ReferencialCartesiano referencial, Matrix world, Matrix view, Matrix projection)
        {
            Model model = referencial.modelo;
            List<Matrix> meshWorldMatrices = referencial.Matrizes(cameraCorrente.posicaoCamera, world);

            for (int index = 0; index < model.Meshes.Count; index++)
            {
                ModelMesh mesh = model.Meshes[index];
                Matrix worldMesh = mesh.ParentBone.Transform * meshWorldMatrices[index] * world;

                DrawMesh(mesh, worldMesh, view, projection);
            }
        }

        /// <summary>
        /// Desenha um conjunto de vetores de projeção
        /// </summary>
        /// <param name="conjuntoVetoresProjecao">Objeto que representa o conjunto de vetores de projeção</param>
        /// <param name="world">matriz do referencial do objeto</param>
        /// <param name="view">matriz da câmera</param>
        /// <param name="projection">matriz da projeção</param>
        private void DrawConjuntoVetoresProjecao(ConjuntoVetoresProjecao conjuntoVetoresProjecao, Matrix world, Matrix view, Matrix projection)
        {
            Model model = conjuntoVetoresProjecao.modeloFixo;
            List<Matrix> meshWorldMatrices = conjuntoVetoresProjecao.Matrizes(cameraCorrente.posicaoCamera, world);

            int primeiroIndexMatrizesZt = model.Meshes.Count;

            for (int index = 0; index < model.Meshes.Count; index++)
            {
                ModelMesh mesh = model.Meshes[index];
                Matrix worldMesh = mesh.ParentBone.Transform * meshWorldMatrices[index] * world;

                DrawMesh(mesh, worldMesh, view, projection);
            }

            model = conjuntoVetoresProjecao.modeloVetorZt;
                        
            for (int index = 0; index < model.Meshes.Count; index++)
            {
                ModelMesh mesh = model.Meshes[index];
                Matrix worldMesh = mesh.ParentBone.Transform * meshWorldMatrices[primeiroIndexMatrizesZt + index] * world;

                DrawMesh(mesh, worldMesh, view, projection);
            }
        }

        /// <summary>
        /// Desenha um plano
        /// </summary>
        /// <param name="plano">plano a ser desenhado</param>
        /// <param name="world">matriz do referencial do objeto</param>
        /// <param name="view">matriz da câmera</param>
        /// <param name="projection">matriz da projeção</param>
        private void DrawPlano(Plano plano, Matrix world, Matrix view, Matrix projection)
        {
            Model model = plano.modelo;

            Vector3 pontoPartePlano = new Vector3(0.0f, 0.0f, 0.0f);

            for (float i = -23f; i < 34f; i++)
            {
                pontoPartePlano.Z = -i;

                for (float j = 0; j < 53; j++)
                {
                    pontoPartePlano.Y = j;
                    
                    foreach (ModelMesh mesh in model.Meshes)
                    {
                        DrawMesh(mesh, Matrix.CreateTranslation(pontoPartePlano) * world, view, projection, 0.5f);
                    }
                }
            }
        }

        /// <summary>
        /// Índice da ajuda dos comandos de teclado da simulação
        /// </summary>
        private void IndiceDaAjuda()
        {
            Console.WriteLine("");
            Console.WriteLine("---------------------------------------");
            Console.WriteLine("----- Índice da ajuda da simulação ----");
            Console.WriteLine("---------------------------------------");
            Console.WriteLine("");
            Console.WriteLine("F2 Câmera e suas coordenadas");
            Console.WriteLine("F3 Comandos de teclado das juntas do braço robô");
            Console.WriteLine("F4 Comandos para exibição/ocultação de partes do braço robô");
            Console.WriteLine("F5 Comandos para exibição/ocultação de referenciais cartesianos (frames)");
            Console.WriteLine("F6 Comandos para exibição/ocultação de planos verticais");
            Console.WriteLine("F7 Comandos para o conjunto de vetores de projeção");
            Console.WriteLine("F8 Sobre...");
        }

        /// <summary>
        /// Ajuda dos comandos da câmera e de suas coordenadas
        /// </summary>
        private void AjudaCamera()
        {
            Console.WriteLine("");
            Console.WriteLine("Câmera e suas coordenadas:");
            Console.WriteLine("-------------------------");
            Console.WriteLine("L alterna entre câmera do braço robô e dos LEDs");
            Console.WriteLine("Shift + L ativa/desativa a mudança automática de câmera");
            Console.WriteLine("NumPad 7 alterna entre câmera do pulso da garra e braço robô");
            Console.WriteLine("NumPad 9 alterna entre câmera do ponto da garra e braço robô");
            Console.WriteLine("+ Aproxima câmera");
            Console.WriteLine("- Afasta câmera");
            Console.WriteLine("R Reseta as coordenadas da câmera corrente para a");
            Console.WriteLine("  posição padrão");
            Console.WriteLine("Direcional para cima eleva a posição da câmera em");
            Console.WriteLine("                     torno da posição alvo");
            Console.WriteLine("Shift + Direcional para cima eleva a posição alvo");
            Console.WriteLine("                             da câmera");
            Console.WriteLine("Direcional para baixo abaixa a posição da câmera em");
            Console.WriteLine("                      torno da posição alvo");
            Console.WriteLine("Shift + Direcional para baixo abaixa a posição alvo");
            Console.WriteLine("                              da câmera");
            Console.WriteLine("Direcional para direita move azimute da câmera para direita");
            Console.WriteLine("Direcional para esquerda move azimute da câmera para esquerda");
            Console.WriteLine("");
        }

        /// <summary>
        /// Ajuda dos comandos de teclado das juntas do braço robô
        /// </summary>
        private void AjudaComandosDasJuntas()
        {
            Console.WriteLine("");
            Console.WriteLine("Comandos de teclado das juntas do braço robô:");
            Console.WriteLine("--------------------------------------------");
            Console.WriteLine("A/Z move a junta 0 (base giratória)");
            Console.WriteLine("S/X move a junta 1");
            Console.WriteLine("D/C move a junta 2");
            Console.WriteLine("F/V move a junta 3");
            Console.WriteLine("G/B move a junta 4 (pulso)");
            Console.WriteLine("H/N abre/fecha a garra");
            Console.WriteLine("");
        }

        /// <summary>
        /// Ajuda dos comandos de exibição das partes do braço robô
        /// </summary>
        private void AjudaPartesBracoRobo()
        {
            Console.WriteLine("");
            Console.WriteLine("Comandos para exibição/ocultação de partes do braço robô:");
            Console.WriteLine("--------------------------------------------------------");
            Console.WriteLine("9 Habilita 1 componente do braço robô para visualização,");
            Console.WriteLine("   começando da garra");
            Console.WriteLine("0 Habilita 1 componente do braço robô para visualização,");
            Console.WriteLine("   começando da base fixa");
            Console.WriteLine("P reseta a visualização para todos os componentes do braço robô");
        }

        /// <summary>
        /// Ajuta dos comandos de exibição dos referenciais cartesianos (frames)
        /// </summary>
        private void AjudaReferenciaisCartesianos()
        {
            Console.WriteLine("");
            Console.WriteLine("Comandos para exibição/ocultação de referenciais cartesianos (frames):");
            Console.WriteLine("---------------------------------------------------------------------");
            Console.WriteLine("1 Exibe/oculta o referencial cartesiano da base fixa (frame 0)");
            Console.WriteLine("2 Exibe/oculta o referencial cartesiano da base giratória (frame 1)");
            Console.WriteLine("3 Exibe/oculta o referencial cartesiano do segmento L1 (frame 2)");
            Console.WriteLine("4 Exibe/oculta o referencial cartesiano do segmento L2 (frame 3)");
            Console.WriteLine("5 Exibe/oculta o referencial cartesiano do segmento L3 (frame 4)");
            Console.WriteLine("6 Exibe/oculta o referencial cartesiano do pulso da garra (frame 5)");
            Console.WriteLine("7 Exibe/oculta o referencial cartesiano do ponto da garra (ponto");
            Console.WriteLine("  calculado por parâmetros de Denavit-Hatenberg)");
            Console.WriteLine("Shift + 7 Exibe/oculta o referencial cartesiano do ponto da garra (ponto exato)");
            Console.WriteLine("TAB Alterna entre os referenciais exatos e os planificados (quando visíveis),");
            Console.WriteLine("    alternando os parâmetros de Denavit-Hatenberg correspondentes.");
            Console.WriteLine("Shift + P Oculta todos os referenciais, planos e conjunto de vetores");
            Console.WriteLine("          de projeção.");
        }

        /// <summary>
        /// Ajuda dos comandos de exibição dos planos verticais
        /// </summary>
        private void AjudaPlanosVerticais()
        {
            Console.WriteLine("");
            Console.WriteLine("Comandos para exibição/ocultação de planos verticais:");
            Console.WriteLine("----------------------------------------------------");
            Console.WriteLine("Shift + 1 Exibe/oculta o plano que corta o braço robô ao meio");
            Console.WriteLine("          pelo ponto referencial da base fixa");
            Console.WriteLine("Shift + 2 Exibe/oculta o plano que corta a garra ao meio pelo");
            Console.WriteLine("          ponto referencial da mesma (plano paralelo ao");
            Console.WriteLine("          anterior)");
            Console.WriteLine("Shift + 3 Exibe/oculta o plano oblíquo que passa pelo ponto");
            Console.WriteLine("          referencial da base fixa e pelo ponto da garra");
            Console.WriteLine("Shift + P Oculta todos os referenciais, planos e conjunto de vetores");
            Console.WriteLine("          de projeção.");
        }

        /// <summary>
        /// Ajuda dos comandos para exibir e manipular os vetores de projeção
        /// </summary>
        private void AjudaConjuntoDeVetoresDeProjecao()
        {
            Console.WriteLine("");
            Console.WriteLine("Comandos para o conjunto de vetores de projeção:");
            Console.WriteLine("-----------------------------------------------");
            Console.WriteLine("8 Exibe/oculta o conjunto de vetores de projeção da posição alvo no");
            Console.WriteLine("  plano vertical que corta o braço ao meio robô pela origem da base");
            Console.WriteLine("  fixa. Apenas para demonstração.");
            Console.WriteLine("Numpad 4 Gira o vetor Zt para perto do vetor Ztl, quando visível");
            Console.WriteLine("Numpad 6 Gira o vetor Zt para longe do vetor Ztl (e para perto do vetor M),");
            Console.WriteLine("         quando visível.");
            Console.WriteLine("Shift + P Oculta todos os referenciais, planos e conjunto de vetores");
            Console.WriteLine("          de projeção.");
            Console.WriteLine("Numpad 5 Ativa modo de digitação de posição alvo. O conjunto de vetores");
            Console.WriteLine("         desaparece na primeira vez, pois ainda não foi digitada a posição");
            Console.WriteLine("         alvo.");
            Console.WriteLine("Numpad 0 Habilita a digitação da posição alvo no console. É preciso");
            Console.WriteLine("         clicar (dar foco) no console para digitar a posição alvo.");
            Console.WriteLine("Numpad 8 Exibe no console a posição alvo do vetor Zt.");
        }
        /// <summary>
        /// Sobre a simulação do braço robô MRB-5GL
        /// </summary>
        private void Sobre()
        {
            Console.WriteLine("");
            Console.WriteLine("Simulação do Braço Robô MRB-5GL - Simulates MRB-5GL, a 5-DOF robot arm prototype");
            Console.WriteLine("Copyright (C) 2019 Amaro Duarte de Paula Neto");
            Console.WriteLine("");
            Console.WriteLine("This program is free software: you can redistribute it and/or modify");
            Console.WriteLine("it under the terms of the GNU General Public License as published by");
            Console.WriteLine("the Free Software Foundation, either version 3 of the License, or");
            Console.WriteLine("(at your option) any later version.");
            Console.WriteLine("");
            Console.WriteLine("This program is distributed in the hope that it will be useful,");
            Console.WriteLine("but WITHOUT ANY WARRANTY; without even the implied warranty of");
            Console.WriteLine("MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the");
            Console.WriteLine("GNU General Public License for more details.");
            Console.WriteLine("");
            Console.WriteLine("You should have received a copy of the GNU General Public License");
            Console.WriteLine("along with this program.  If not, see <https://www.gnu.org/licenses/>.");
            Console.WriteLine("");
            Console.WriteLine("e-mail: amaro.net80@gmail.com");
            Console.WriteLine("");
        }
    }
}
