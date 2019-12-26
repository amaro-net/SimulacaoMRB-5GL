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
using Microsoft.Xna.Framework;
using System.Collections.Generic;
using System.IO.Ports;
using System;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Content;
//using System.Threading;
using System.Timers;

#pragma warning disable 168

namespace BracoRobo.Classes
{
    /// <summary>
    /// Delegate para conectar notificações de evento de recebimento do comando LED
    /// </summary>
    /// <param name="sender">Objeto que enviou a notificação</param>
    /// <param name="e">argumentos do evento</param>
    public delegate void ComandoLEDRecebidoEventHandler(object sender, EventArgs e);
    
    /// <summary>
    /// Delegate para conectar notificações de evento de recebimento de quaisquer comandos 
    /// de acionamento de servos (JST, RPS, CTZ, GA ou GF)
    /// </summary>
    /// <param name="sender">Objeto que enviou a notificação</param>
    /// <param name="e">argumentos do evento</param>
    public delegate void ComandoAcionamentoServoRecebidoEventHandler(object sender, EventArgs e);

    /// <summary>
    /// Enumaração para o flag para os comandos que foram acionados. Utilizado para rotinas que requeiram execução em tempo real,
    /// tal como o acionamento dos servomotores
    /// </summary>
    public enum ComandoProto {
        /// <summary>
        /// nenhum comando
        /// </summary>
        cmdNone = 0, 
        /// <summary>
        /// Comando LED
        /// </summary>
        cmdLED = 1, 
        /// <summary>
        /// comando GA (Garra Abrir)
        /// </summary>
        cmdGA = 2, 
        /// <summary>
        /// comando GF (Garra Fechar)
        /// </summary>
        cmdGF = 3, 
        /// <summary>
        /// Comando CTZ (CenTraliZar)
        /// </summary>
        cmdCTZ = 4, 
        /// <summary>
        /// Comando JST (Junta Setar Target)
        /// </summary>
        cmdJST = 5, 
        /// <summary>
        /// Comando TMX (Target MáXimo)
        /// </summary>
        cmdTMX = 6, 
        /// <summary>
        /// Comando TMN (Target MíNimo)
        /// </summary>
        cmdTMN = 7, 
        /// <summary>
        /// Comando T90 (Target a 90 graus, para o comando CTZ) 
        /// </summary>
        cmdT90 = 8, 
        /// <summary>
        /// Comando TRP (Target RePouso, para o comando RPS)
        /// </summary>
        cmdTRP = 9, 
        /// <summary>
        /// Comando VEL (VELocidade)
        /// </summary>
        cmdVEL = 10, 
        /// <summary>
        /// Comando ACL (ACeLeração)
        /// </summary>
        cmdACL = 11, 
        /// <summary>
        /// Comando FRS (Feedback de Rastreamento dos Servos)
        /// </summary>
        cmdFRS = 12, 
        /// <summary>
        /// Comando CSB (Comandos de Servos Bloqueantes)
        /// </summary>
        cmdCSB = 13, 
        /// <summary>
        /// Comando EMM (Erro da Mini Maestro)
        /// </summary>
        cmdEMM = 14, 
        /// <summary>
        /// Comando GTP (GeT Position da Mini Maestro)
        /// </summary>
        cmdGTP = 15, 
        /// <summary>
        /// Comando GMS (Get Moving State da Mini Maestro)
        /// </summary>
        cmdGMS = 16, 
        /// <summary>
        /// Comando STT (STaTus)
        /// </summary>
        cmdSTT = 17, 
        /// <summary>
        /// Comando RST (ReSeT)
        /// </summary>
        cmdRST = 18, 
        /// <summary>
        /// Sinal IN1 (INicialização de posição 1)
        /// </summary>
        cmdIN1 = 19, 
        /// <summary>
        /// Sinal IN2 (INicialização de posição 2)
        /// </summary>
        cmdIN2 = 20, 
        /// <summary>
        /// Comando ECH (ECo Habilitado)
        /// </summary>
        cmdECH = 21, 
        /// <summary>
        /// Comando RPS (RePouSo)
        /// </summary>
        cmdRPS = 22,
        /// <summary>
        /// Comando DSL (DeSLiga)
        /// </summary>
        cmdDSL = 23,
        /// <summary>
        /// Comando PRT (PaRada Total)
        /// </summary>
        cmdPRT = 24
    }
    /// <summary>
    /// Enumeração para o flag para indicar se o comando de acionamento dos servos acionados aciona 0, 1 ou mais servos
    /// </summary>
    public enum TipoMultiplicidadeServos { 
        /// <summary>
        /// Nenhum servo acionado
        /// </summary>
        mtpNenhumServo = 0, 
        /// <summary>
        /// Apenas um servo acionado
        /// </summary>
        mtpApenasUmServo = 1, 
        /// <summary>
        /// 2 ou mais servos acionados
        /// </summary>
        mtpVariosServos = 2
    }
    /// <summary>
    /// Enumeração para o flag para indicar que tipo de feedback os comandos de acionamento dos servos irão
    /// enviar para a porta serial
    /// </summary>
    public enum TipoFeedbackServos {
        /// <summary>
        /// Sem feedback
        /// </summary>
        fdbSemFeedback = 0, 
        /// <summary>
        /// Tempos (targets) de posição corrente dos servos
        /// </summary>
        fdbTemposDosServos = 1, 
        /// <summary>
        /// Sinal indicativo de movimento
        /// </summary>
        fdbSinalDeMovimento = 2
    }

    /// <summary>
    /// Classe para representar um robô em sua totalidade.
    /// </summary>
    public class Robo : ComponenteFisico
    {
        /// <summary>
        /// Evento que irá notificar o recebimento de um comando LED
        /// </summary>
        public event ComandoLEDRecebidoEventHandler ComandoLEDRecebido;
        /// <summary>
        /// Evento que irá notificar o recebimento de quaisquer comandos de quaisquer comandos 
        /// de acionamento de servos (JST, RPS, CTZ, GA ou GF)
        /// </summary>
        public event ComandoAcionamentoServoRecebidoEventHandler ComandoAcionamentoServoRecebido;

        /// <summary>
        /// Invoca o evento ComandoLEDRecebido. Chamado sempre que for recebido um comando LED com parâmetros
        /// </summary>
        /// <param name="e">parâmetros do evento</param>
        protected virtual void OnComandoLEDRecebido(EventArgs e)
        {
            if (ComandoLEDRecebido != null)
            {
                ComandoLEDRecebido(this, e);
            }
        }

        /// <summary>
        /// Invoca o evento ComandoAcionamentoServoRecebido. Chamado sempre que for recebido quaisquer comandos 
        /// de acionamento de servos (JST, RPS, CTZ, GA ou GF)
        /// </summary>
        /// <param name="e"></param>
        protected virtual void OnComandoAcionamentoServoRecebido(EventArgs e)
        {
            if (ComandoAcionamentoServoRecebido != null)
            {
                ComandoAcionamentoServoRecebido(this, e);
            }
        }

        /// <summary>
        /// Base do robô
        /// </summary>
        public Base baseFixa;
        /// <summary>
        /// Motor entre a base do robô e a base giratória
        /// </summary>
        public Servomotor motorBase;
        /// <summary>
        /// Base giratória
        /// </summary>
        public Base baseGiratoria;
        /// <summary>
        /// Motor entre a base giratória e o segmento de braço 1
        /// </summary>
        public Servomotor motor1;
        /// <summary>
        /// Segmento de braço 1
        /// </summary>
        public SegmentoDeBraco bracoL1;
        /// <summary>
        /// Motor entre os segmentos de braço 1 e 2
        /// </summary>
        public Servomotor motor2;
        /// <summary>
        /// Segmento de braço 2
        /// </summary>
        public SegmentoDeBraco bracoL2;
        /// <summary>
        /// Motor entre os segmentos de braço 2 e 3
        /// </summary>
        public Servomotor motor3;
        /// <summary>
        /// Segmento de braço 3
        /// </summary>
        public SegmentoDeBraco bracoL3;
        /// <summary>
        /// Motor entre o segmento de braço 3 e a garra. É o motor que irá girar a garra.
        /// </summary>
        public Servomotor motorGiroGarra;
        /// <summary>
        /// Garra do robô
        /// </summary>
        public GarraMK2 garra;


        /// <summary>
        /// Buffer contendo o estado dos leds
        /// </summary>
        public bool[] bufferLeds;

        /// <summary>
        /// Objeto que representa a placa Mini Maestro 24 que controla os servos do braço robô
        /// </summary>
        public PlacaMiniMaestro24 placaMiniMaestro24;


        /* Atributos da porta serial do robô */

        /// <summary>
        /// Objeto para a porta serial que irá receber os comandos do robô, bem como mandar para o controle
        /// quaisquer dados referentes ao robô que se fizerem necessários, tais como dados dos sensores, por exemplo.
        /// </summary>
        public SerialPort serialCom;
        /// <summary>
        /// String que possui os comandos aplicados ao robô via porta serial. Esta string irá obter o valor do campo
        /// bfRecebe, quando este tiver recebido todos os caracteres que compõem os comandos do robô.
        /// </summary>
        public String bufferProto = String.Empty;
        /// <summary>
        /// String para receber caracteres via porta serial. O acesso a esta string só deve ocorrer quando todos os caracteres
        /// forem recebidos.
        /// </summary>
        public String bfRecebe = String.Empty;
        /// <summary>
        /// Variável contendo os caracteres recém recebidos da porta serial
        /// </summary>
        private String recChars;
        /// <summary>
        /// Variável que indica quando os caracteres de um comando estão sendo recebidos, ou seja,
        /// quando um abre-colchete ([) foi recebido. Valor false indica que nenhum [ foi recebido.
        /// Esta variável vai a false, também, quando um fecha-colchete (]) é recebido.
        /// </summary>
        bool flagProtocolo = false;
        /// <summary>
        /// Indica se bufferProto já recebeu um pacote completo.
        /// </summary>
        bool DataReady = false;
        /// <summary>
        /// Variável de estado para guiar a execução do método PosicaoRepouso() enquanto os gráficos são atualizados
        /// Por padrão, 0 é o estado inicial, 1 é o estado final, e os demais serão os intermediários.
        /// </summary>
        public int estadoPosicaoRepouso = 0;

        /// <summary>
        /// Variável para indicar se os comandos JST, RPS, CTZ, GA e GF irão ou não mandar para a
        /// porta serial as posições que os servos envolvidos assumem durante os movimentos dos mesmos.
        /// Valor zero indica que os comandos não irão enviar feedback durante o movimento dos
        /// servos.
        /// </summary>
        public TipoFeedbackServos feedbackRastrServos = TipoFeedbackServos.fdbTemposDosServos;
        /// <summary>
        /// Comando reconhecido pelo protocolo do robô
        /// </summary>
        public ComandoProto comando = ComandoProto.cmdNone;
        /// <summary>
        /// Último comando acionado que move os servos
        /// </summary>
        public ComandoProto ultCmdMovServos = ComandoProto.cmdNone;
        /// <summary>
        /// String referente ao último comando de acionamento dos servos executado
        /// </summary>
        public String strUltCmdMovServos = String.Empty;
        /// <summary>
        /// Determina se será zero, um ou vários servos a serem acionados
        /// </summary>
        public TipoMultiplicidadeServos tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpNenhumServo;
        /// <summary>
        /// Indica qual canal do servo será ativado (0 a 5).
        /// </summary>
        public byte canalCmd1ServoNaoBloq;
        /// <summary>
        /// Flag para indicar se uma parada total foi solicitada via comando PRT.
        /// </summary>
        public bool paradaTotalSolicitada = false;
        /// <summary>
        /// Variável para indicar se os comandos dos servos irão ou não impedir que novos comandos
        /// sejam recebidos enquanto os servos estiverem se movendo.
        /// </summary>
        public bool comandosServosBloqueantes = false;
        /// <summary>
        /// Variável para indicar se os caracteres recebidos pela UART serão enviados de volta.
        /// Ativado por padrão para permitir melhor interação com uso de programas de terminal
        /// de porta serial, tal como o Tera Term.
        /// </summary>
        public bool ecoCaracteresAtivado = true;
        /// <summary>
        /// Variável para ser usada como índice do servo a ser configurado.
        /// </summary>
        private UInt16 indexServo;
        /// <summary>
        /// Vetor contendo os endereços dos servos
        /// </summary>
        Servomotor[] servos;
        /// <summary>
        /// Vetor contendo os valores temporários dos servos
        /// </summary>
        Servomotor[] servosTemp;

        /// <summary>
        /// Constante para indicar o delay entre o acionamento de um comando de acionamento dos servos e
        /// o início do feedback.
        /// </summary>
        public const int DELAY_MS_RESPOSTA = 500;
        /// <summary>
        /// Variável utilizada para guardar a resposta a um comando
        /// </summary>
        public String resposta;
        /// <summary>
        /// Atributo para guardar provisoriamente a configuração do feedback dos servos
        /// </summary>
        protected TipoFeedbackServos frsTemp;
        /// <summary>
        /// Vetor que guarda os targets para uso no comando SetMultipleTargets() da placa mini maestro 24
        /// </summary>
        public float[] vetorTargets;
        /// <summary>
        /// Atributo que serve para contagem de tempo
        /// </summary>
        Timer timer;
        /// <summary>
        /// Último caractere recebido pela porta serial. Armazena apenas caracteres alfanuméricos, abre colchete ([) ou
        /// fecha colchete (])
        /// </summary>
        public String ultimoCharRecebido;
        /// <summary>
        /// Define um canal indefinido da mini maestro.
        /// </summary>
        private byte CANAL_INDEFINIDO = 255;

          ///////////////////////////////
         ////////// MÉTODOS //////////// 
        /////////////////////////////// 


        /// <summary>
        /// Construtor do robô. Aqui serão inicializados todos os componentes físicos do robô.
        /// </summary>
        public Robo(string[] args = null)
        {
            bool podeConectarPortaSerial = true;
            String portName = String.Empty;

            ultimoCharRecebido = "?"; // para indicar que nenhum caractere foi recebido ainda

            this.servos = new Servomotor[6];
            this.servosTemp = new Servomotor[6];
            this.timer = new Timer(2000);
            this.timer.AutoReset = false; // true para o timer estourar sempre que contar o tempo, false para o timer
            // desabilitar após o primeiro estouro.

            for (int i = 0; i < servosTemp.Length; i++)
            {
                this.servosTemp[i] = new Servomotor();
            }

            vetorTargets = new float[6];

            // Porta serial
            serialCom = new SerialPort();
            serialCom.DataReceived += new SerialDataReceivedEventHandler(serialCom_DataReceived);

            if (serialCom.IsOpen) serialCom.Close();

            String[] nomesPortasSeriais = SerialPort.GetPortNames();

            if (nomesPortasSeriais.Length > 0)
            {
                if (args != null && args.Length > 0)
                {
                    String porta = args[0];
                    bool portaExiste = false;
                                        
                    int i = 0;
                    while (!portaExiste && i < nomesPortasSeriais.Length)
                    {
                        String p = nomesPortasSeriais[i++];
                        portaExiste = porta.Equals(p);
                    }

                    podeConectarPortaSerial = portaExiste;

                    if (podeConectarPortaSerial)
                        portName = porta;
                    else
                        Console.WriteLine("Porta "+porta+" não existe nesta máquina.");
                }
                else if (nomesPortasSeriais.Length > 1)
                {
                    Console.WriteLine("Esta máquina possui as seguintes portas:");
                    int i = 0;
                    if (nomesPortasSeriais.Length <= 21)
                    {
                        foreach (string str in nomesPortasSeriais)
                        {
                            i++;
                            Console.WriteLine(i.ToString() + ") " + str);
                        }
                    }
                    else
                    {
                        int tamStrMaxPortaCom = 0;
                        String strAux = "";
                        foreach (String str in nomesPortasSeriais)
                        {
                            if (tamStrMaxPortaCom < str.Length)
                            {
                                tamStrMaxPortaCom = str.Length;
                                strAux = str;
                            }
                        }

                        int tamStrIdxMax = strAux.Length;

                        int tamStrIdxMaisPortaMax = tamStrIdxMax + 2 + tamStrMaxPortaCom;

                        int idxStr = 0;
                        for(i = 0; i < 21; i++)
                        {
                            String strIdxMaisPorta = (i + 1).ToString() + ") " + nomesPortasSeriais[i];

                            Console.Write(strIdxMaisPorta);
                            
                            idxStr = i + 21;
                            while (idxStr < nomesPortasSeriais.Length)
                            {
                                int tamStrIdxMaisPorta = strIdxMaisPorta.Length;
                                String strIdx = (idxStr + 1).ToString();
                                String strPortaSerial = nomesPortasSeriais[idxStr];
                                String strIdxMaisPorta2c = strIdx + ") " + strPortaSerial;                                

                                int qtdEspacos = tamStrIdxMaisPortaMax - tamStrIdxMaisPorta;
                                while (qtdEspacos > 0)
                                {
                                    Console.Write(" ");
                                    qtdEspacos--;
                                }

                                Console.Write(" " + strIdxMaisPorta2c);
                                idxStr = idxStr + 21;
                                strIdxMaisPorta = strIdxMaisPorta2c;
                            }
                            Console.WriteLine("");
                        }

                    }
                    Console.WriteLine("Qual porta a simulação deve usar? (digite um número de 1 a " + i.ToString() + ", ou 0 para não utilizar nenhuma porta serial)");
                    String opt = Console.ReadLine();
                    int opcao = 0;

                    try
                    {
                        opcao = int.Parse(opt);
                    }
                    catch (ArgumentNullException e)
                    {
                        Console.WriteLine("Erro: valor nulo detectado.");
                        podeConectarPortaSerial = false;
                    }
                    catch (ArgumentException e)
                    {
                        Console.WriteLine("Erro: opção inválida.");
                        podeConectarPortaSerial = false;
                    }
                    catch (OverflowException e)
                    {
                        Console.WriteLine("Valor digitado é maior que " + int.MaxValue.ToString() + " ou menor que " + int.MinValue.ToString());
                        podeConectarPortaSerial = false;
                    }
                    catch (Exception e)
                    {
                        Console.WriteLine(e.ToString());
                        podeConectarPortaSerial = false;
                    }


                    if (opcao == 0)
                    {
                        podeConectarPortaSerial = false;
                    }
                    else
                    {
                        portName = nomesPortasSeriais[opcao-1];
                        podeConectarPortaSerial = true;
                    }
                }
                else
                {
                    Console.WriteLine("A porta " + nomesPortasSeriais[0] + " é a única porta serial encontrada. Deseja utilizar esta porta? (S ou N)");
                    String opt = Console.ReadLine();
                    if (opt.Equals("S"))
                    {
                        portName = nomesPortasSeriais[0];
                        podeConectarPortaSerial = true;
                    }
                    else
                    {
                        podeConectarPortaSerial = false;
                    }
                }

                if (podeConectarPortaSerial)
                {
                    serialCom.PortName = portName;
                    serialCom.BaudRate = 9600;
                    serialCom.Parity = Parity.None;
                    serialCom.DataBits = 8;
                    serialCom.StopBits = StopBits.One;
                    serialCom.NewLine = "\n\r";

                    try
                    {
                        serialCom.Open();
                        Console.WriteLine("Porta "+serialCom.PortName+" conectada com sucesso.");
                    }
                    catch
                    {
                        Console.WriteLine("Não foi possível abrir a porta selecionada (" + serialCom.PortName + ").");
                    }
                    finally
                    {
                    }
                }
                else
                {
                    Console.WriteLine("Nenhuma porta serial será utilizada durante a simulação.");
                }

            }
            else
            {
                Console.WriteLine("Nenhuma porta serial foi encontrada.");
            }

            Console.WriteLine("Continuando a inicialização da simulação.");


            // LEDS
            bufferLeds = new bool[8];
            for (int i = 0; i < bufferLeds.Length; i++)
            {
                bufferLeds[i] = false;
            }

            // Placa Mini Maestro 24
            this.placaMiniMaestro24 = new PlacaMiniMaestro24();

            // Base fixa
            this.baseFixa = new Base();
            this.baseFixa.posReferencial = new Vector3(0.0f, 0.0f, 0.0f);
            this.baseFixa.anguloGiro = new Vector3(0.0f, 0.0f, 0.0f);
            this.baseFixa.velocidadeTrans = new Vector3(0.0f, 0.0f, 0.0f);
            this.baseFixa.velocidadeAng = new Vector3(0.0f, 0.0f, 0.0f);
            this.baseFixa.aceleracaoTrans = new Vector3(0.0f, 0.0f, 0.0f);
            this.baseFixa.aceleracaoAng = new Vector3(0.0f, 0.0f, 0.0f);
            this.baseFixa.centroMassa = new Vector3(0.0f, 0.084f, 0.0f);

            // Base giratória
            this.baseGiratoria = new Base();            
            this.baseGiratoria.posReferencial = new Vector3(0.0f, 0.6f, 0.0f);
            this.baseGiratoria.centroGiro = new Vector3(0.0f, 0.6f, 0.0f);            

            // Segmento de braço 1
            this.bracoL1 = new SegmentoDeBraco();
            this.bracoL1.posReferencial = new Vector3(0.0f, 0.5174996f, 0.0f);
            this.bracoL1.centroGiro = this.bracoL1.posReferencial;
            this.bracoL1.comprimento = 11.65f;

            // Segmento de braço 2
            this.bracoL2 = new SegmentoDeBraco();
            this.bracoL2.posReferencial = new Vector3(0.0f, 0.679045f, 0.0f);
            this.bracoL2.centroGiro = this.bracoL2.posReferencial;
            this.bracoL2.comprimento = 5.825f;

            // Segmento de braço 3
            this.bracoL3 = new SegmentoDeBraco();
            this.bracoL3.posReferencial = new Vector3(0.0f, 0.6949995f, 0.0f);
            this.bracoL3.centroGiro = this.bracoL3.posReferencial;
            this.bracoL3.comprimento = 8.633297f;

            // Garra do robô
            this.garra = new GarraMK2();
            this.garra.posReferencial = new Vector3(0.0f, 0.235f, 0.0f);
            this.servos[5] = this.garra.servoDaGarra;
            this.placaMiniMaestro24.canais[5].servo = this.garra.servoDaGarra;

            // Motor entre base fixa e base giratória
            this.motorBase = new Servomotor();
            this.motorBase.posReferencial = new Vector3(0.0f, 7.8f, 0.0f);
            this.motorBase.anguloGiro = new Vector3(0.0f, 180.0f, 0.0f);
            this.motorBase.sigla = "J0";
            this.motorBase.idJST = "A";
            this.motorBase.VelocidadeAngularEixo = new Vector3(0.0f, 1.02f, 0.0f);
            this.motorBase.FixarEm(this.baseFixa);
            this.motorBase.AcoplarAoEixo(this.baseGiratoria, EixoCartesiano.Ypos);
            this.motorBase.cargaDeAtuacao.anguloGiro.Y = 0.0f;
            this.motorBase.corpo.anguloGiro = this.motorBase.anguloGiro;
            this.motorBase.eixo.anguloGiro = this.motorBase.anguloGiro;
            this.motorBase.anguloGiroEixoInicial = new Vector3(0.0f, 0.0f, 0.0f);
            this.motorBase.AnguloGiroEixo = new Vector3(0.0f, 0.0f, 0.0f);
            this.motorBase.anguloMin = -90.0f; // graus em relação ao eixo z, em torno de y
            this.motorBase.anguloMax = 100.0f; // graus em relação ao eixo z, em torno de y
            this.motorBase.AnguloCorrente = 0.0f;
            this.motorBase.proporcaoInversaTmpPulsoParaAngulo = false;
            this.servos[0] = this.motorBase;
            this.placaMiniMaestro24.canais[0].servo = this.motorBase;            

            // Motor entre base giratória e segmento de braço 1
            this.motor1 = new Servomotor();
            this.motor1.sigla = "J1";
            this.motor1.idJST = "B";
            this.motor1.posReferencial = new Vector3(3.7f, 9.01098f, -0.05f);
            this.motor1.anguloGiro = new Vector3(0.0f, 180.0f, -90.0f);
            this.motor1.VelocidadeAngularEixo = new Vector3(0.0f, 1.02f, 0.0f);
            this.motor1.FixarEm(this.baseGiratoria);
            this.motor1.AcoplarAoEixo(this.bracoL1, EixoCartesiano.Xpos);
            this.motor1.cargaDeAtuacao.anguloGiro.X = 180f; // para colocar o segmento de pé
            this.motor1.anguloGiroEixoInicial = this.motor1.anguloGiro;
            this.motor1.AnguloGiroEixo = this.motor1.anguloGiroEixoInicial;
            this.motor1.corpo.anguloGiro = this.motor1.anguloGiro;
            this.motor1.eixo.anguloGiro = this.motor1.AnguloGiroEixo;
            this.motor1.anguloMin = -90; // graus em relação ao eixo y, em torno de x
            this.motor1.anguloMax = 40; // graus em relação ao eixo y, em torno de x
            this.motor1.AnguloCorrente = 0.0f;
            this.motor1.difAnguloCorrenteDH = 90f;
            this.motor1.proporcaoInversaTmpPulsoParaAngulo = false;
            this.servos[1] = this.motor1;
            this.placaMiniMaestro24.canais[1].servo = this.motor1;
            
            // Motor entre os segmentos de braço 1 e 2
            this.motor2 = new Servomotor();
            this.motor2.sigla = "J2";
            this.motor2.idJST = "C";
            this.motor2.posReferencial = new Vector3(-4.1f, 11.675f, 0.0f);
            this.motor2.anguloGiro = new Vector3(0.0f, -90.0f, -90.0f);
            this.motor2.VelocidadeAngularEixo = new Vector3(0.0f, 1.02f, 0.0f);
            this.motor2.corpo.anguloGiro = this.motor2.anguloGiro;
            this.motor2.eixo.anguloGiro = this.motor2.anguloGiro;
            this.motor2.FixarEm(this.bracoL1);
            this.motor2.AcoplarAoEixo(this.bracoL2, EixoCartesiano.Xpos);
            this.motor2.cargaDeAtuacao.anguloGiro.X = 180.0f;
            this.motor2.anguloGiroEixoInicial = new Vector3(180.0f, 0.0f,this.motor2.cargaDeAtuacao.anguloGiro.Z);
            this.motor2.AnguloGiroEixo = this.motor2.anguloGiroEixoInicial;
            this.motor2.anguloMin = -133; // graus em relação ao eixo y, em torno de x
            this.motor2.anguloMax = 0; // graus em relação ao eixo y, em torno de x
            this.motor2.AnguloCorrente = 0.0f;
            this.motor2.sinalPropagacaoAnguloCorrente = -1.0f;
            this.motor2.proporcaoInversaTmpPulsoParaAngulo = false;
            this.servos[2] = this.motor2;
            this.placaMiniMaestro24.canais[2].servo = this.motor2;
            

            // Motor entre os segmentos de braço 2 e 3
            this.motor3 = new Servomotor();
            this.motor3.sigla = "J3";
            this.motor3.idJST = "D";
            this.motor3.posReferencial = new Vector3(-0.6f, 5.8f, 0.0f);
            this.motor3.anguloGiro = new Vector3(0.0f, 90.0f, 90.0f);
            this.motor3.corpo.anguloGiro = this.motor3.anguloGiro;
            this.motor3.eixo.anguloGiro = this.motor3.anguloGiro;
            this.motor3.VelocidadeAngularEixo = new Vector3(0.0f, 1.02f, 0.0f);
            this.motor3.FixarEm(this.bracoL2);
            this.motor3.AcoplarAoEixo(this.bracoL3, EixoCartesiano.Xneg);
            this.motor3.cargaDeAtuacao.anguloGiro.X = 90.0f;
            this.motor3.anguloGiroEixoInicial = this.motor3.anguloGiro;
            this.motor3.AnguloGiroEixo = this.motor3.anguloGiroEixoInicial;
            this.motor3.anguloMin = -126; // graus em relação ao eixo y, em torno de x
            this.motor3.anguloMax = 74; // graus em relação ao eixo y, em torno de x
            this.motor3.AnguloCorrente = 0.0f;
            this.motor3.difAnguloCorrenteDH = 90f;
            this.motor3.sinalPropagacaoAnguloCorrente = -1.0f;
            this.motor3.proporcaoInversaTmpPulsoParaAngulo = true;
            this.servos[3] = this.motor3;
            this.placaMiniMaestro24.canais[3].servo = this.motor3;

            // Motor entre o segmento de braço 3 e a garra
            this.motorGiroGarra = new Servomotor();
            this.motorGiroGarra.sigla = "J4";
            this.motorGiroGarra.idJST = "E";
            this.motorGiroGarra.posReferencial = new Vector3(-0.8f, 7.95f, 0.45f);
            this.motorGiroGarra.anguloGiro = new Vector3(0.0f, -90.0f, 0.0f);
            this.motorGiroGarra.anguloGiroEixoInicial = new Vector3(0.0f, 90.0f, 0.0f);
            this.motorGiroGarra.AnguloGiroEixo = this.motorGiroGarra.anguloGiroEixoInicial;
            this.motorGiroGarra.VelocidadeAngularEixo = new Vector3(0.0f, 1.0f, 0.0f);
            this.motorGiroGarra.corpo.anguloGiro = this.motorGiroGarra.anguloGiro;
            this.motorGiroGarra.eixo.anguloGiro = this.motorGiroGarra.anguloGiro;
            this.motorGiroGarra.FixarEm(this.bracoL3);
            this.motorGiroGarra.AcoplarAoEixo(this.garra, EixoCartesiano.Ypos);
            this.motorGiroGarra.cargaDeAtuacao.anguloGiro.Y = -90.0f;
            this.motorGiroGarra.anguloMin = -90.0f; // graus em relação ao eixo z, em torno de y
            this.motorGiroGarra.anguloMax = 90.0f; // graus em relação ao eixo z, em torno de y
            this.motorGiroGarra.AnguloCorrente = 0.0f;
            this.motorGiroGarra.proporcaoInversaTmpPulsoParaAngulo = false;
            this.servos[4] = this.motorGiroGarra;
            this.placaMiniMaestro24.canais[4].servo = this.motorGiroGarra;

            IniciarServos();
        }

        /// <summary>
        /// Destrutor da classe Robo
        /// </summary>
        ~Robo()
        {
            if (serialCom.IsOpen == true) serialCom.Close();
        }

        /// <summary>
        /// Sobrecarca da função LoadContent para a classe Robo
        /// </summary>
        public override void LoadContent()
        {
            base.LoadContent();
        }

        /// <summary>
        /// Rotina para capturar o que for recebido da porta serial
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void serialCom_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            recChars = serialCom.ReadExisting();
            bfRecebe += recChars;
        }
        /// <summary>
        /// Obtém um caractere de bfRecebe e retira o mesmo do início. Pode-se dizer que o caractere
        /// obtido é consumido de bfRecebe.
        /// </summary>
        /// <returns>O caractere contido no início de bfRecebe, se este não estiver vazio. Caso contrário, retorna vazio</returns>
        public String GetCharBfRecebe()
        {
            String c;
            if (bfRecebe.Length > 0)
            {
                c = bfRecebe[0].ToString();
                bfRecebe = bfRecebe.Substring(1);
                return c;
            }
            return String.Empty;
        }

        /// <summary>
        /// Função utilizada pela rotina de interrupção para receber os caracteres enviados pela porta serial e armazená-los
        /// em buffer caso sejam enviados entre os caracteres limites da mensagem do protocolo ([ e ]).
        /// </summary>
        public void RecebeCaracteresDoProtocolo()
        {
            
            String rChar = GetCharBfRecebe();
            
            if (rChar.Length > 0)
	        {
                if (char.IsLetterOrDigit(rChar, 0) || rChar.ToCharArray()[0] == '[' || rChar.ToCharArray()[0] == ']')
                {
                    rChar = rChar.ToUpper();
                    ultimoCharRecebido = rChar;
                }

                // Exibição do caractere (se o eco de caracteres estiver ativado)
                if (ecoCaracteresAtivado)
                {
                    if (rChar.ToCharArray()[0] == 8)
                    {
                        serialCom.Write(rChar);
                        serialCom.Write(" ");
                        serialCom.Write(rChar);                        
                    }
                    else if(rChar.Equals("?"))
                    {
                        serialCom.Write(ultimoCharRecebido); // Repete o último caractere recebido
                    }
                    else {
                        serialCom.Write(rChar);
                    }
                }
                if (!flagProtocolo && rChar.Equals("[")) //é o Byte de início do protocolo?
                {
                    //LimparString(bufferProto, TAMANHO_BUFFER_PROTO);
                    //pntBufferProto = &bufferProto; //Salva endereço do buffer no Ponteiro.
                    //*pntBufferProto = rChar; // armazena na posição corrente do buffer o caractere 
                    //++pntBufferProto; // Avança 1 posição do buffer
                    bufferProto = "[";
                    flagProtocolo = true; //Seta flag que indica que o protocolo começou
                }
                else if (flagProtocolo && !rChar.Equals("?")) // se verdadeiro indica que já recebemos o byte de início e
                {                            // devemos armazenar o restante dos caracteres
                    if (rChar.ToCharArray()[0] == 8) // BackSpace (para quando estiver acessando a UART 1 via terminal de porta serial)
                    {
                        //pntBufferProto--; // Recua 1 posição no buffer
                        //if(pntBufferProto == bufferProto) flagProtocolo = 0;
                        //*pntBufferProto = (char)0; // Null - caractere terminador da string "Apaga" o caractere do buffer
                        bufferProto = bufferProto.Substring(0,bufferProto.Length-1);
                        if (bufferProto.Length == 0)
                        {
                            flagProtocolo = false;
                        }
                    }
                    else if (rChar.Equals("]")) // Chegou o byte de finalização do protocolo, logo devemos concluir a recepção dos caracteres
                    {
                        bufferProto += rChar; 
                        flagProtocolo = false; //reinicia flag do protocolo
                        DataReady = true;
                        //Console.WriteLine("Recebido: "+bufferProto);
                    }
                    else if (rChar.Equals("[")) // Se chegou outro byte de início do protocolo
                    {
                        bufferProto = "[";
                    }
                    else
                    {
                        bufferProto += rChar;

                        if (bufferProto.Length >= 39)
                        {
                            flagProtocolo = false; //reinicia flag auxiliar
                            DataReady = true;
                        }
                    }
                } 
	        }
        }

        /// <summary>
        /// Atualiza os eventos do braço robô. Funciona como a função main() em C do braço robô verdadeiro.
        /// </summary>
        /// <param name="gameTime"></param>
        public void Update(GameTime gameTime)
        {
            if(serialCom.IsOpen)
            {
                if (this.estadoPosicaoRepouso == 1)
                {
                    if (DataReady)
                    {
                        DecodificaProtocolo();
                        DataReady = false;
                    }
                    else
                    {
                        if (!comandosServosBloqueantes
                            || (comandosServosBloqueantes && tipoCmdMovServosPorQt == TipoMultiplicidadeServos.mtpNenhumServo)
                           )
                        {
                            RecebeCaracteresDoProtocolo();
                        }
                    }
                }
                else
                {
                    PosicaoRepouso();
                }
                // Resposta e feedback dos servos
                RespEFeedbackMovServos();
            }
        }

        /// <summary>
        /// Função para reconhecer um comando recebido pela porta serial. Esta função reconhece apenas o nome do comando
        /// mas não o formato dos seus parâmetros.
        /// </summary>
        /// <param name="comando">comando String contendo o nome do comando que será reconhecido (JST, LED etc).</param>
        /// <returns>true Se o comando for reconhecido, false caso contrário.</returns>
        private bool Comando(String comando)
        {
            int tam_buffer, tam_comando;
            tam_buffer = bufferProto.Length;
            tam_comando = comando.Length;

            if (tam_buffer < 2)
                return false;

            // se o tamanho do buffer for menor que o tamanho do comando, retorna falso
            if (tam_buffer <= tam_comando)
            {
                return false;
            }

            if(! bufferProto.Substring(1,tam_comando).Equals(comando))
            {
                return false;
            }
           
            // Chegando aqui, o comando contido no buffer é o comando requerido
            return true;
        }

        /// <summary>
        /// Função que envia para a porta serial uma resposta do comando. Normalmente uma resposta de que
        /// o comando foi reconhecido, ou uma resposta contendo valores.
        /// </summary>
        /// <param name="resposta">string contendo a resposta do comando</param>
        public void RespostaComando(String resposta)
        {
            serialCom.WriteLine("");
            serialCom.WriteLine(resposta);
        }
        
        /// <summary>
        /// Envia para a porta serial a resposta de que o formato do comando não foi reconhecido.
        /// Em outras palavras, a resposta que esta função envia significa que o nome do comando
        /// foi reconhecido, mas os seus parâmetros (e/ou o formato dos mesmos) não foram reconhecidos.
        /// </summary>
        /// <param name="comando">String contendo o comando que dará a resposta NAK</param>
        private void RespostaComandoNAK(String comando)
        {
            RespostaComando("["+comando+" NAK]");
        }


        /// <summary>
        /// Envia para a UART a resposta de que houve um erro de conexão entre a Ready For PIC e a placa
        /// de controle dos servos. Em outras palavras, a resposta que esta função envia significa que
        /// o comando e seus parâmetros foram reconhecidos, mas o dispositivo que o comando aciona
        /// não respondeu adequadamente. Esta função só deve ser chamada para comandos que controlem algum
        /// dispositivo, tais como os comandos JST, GA, GF e CTZ, que controlam a placa de controle dos
        /// servos.
        /// </summary>
        /// <param name="comando">String contendo o comando que dará a resposta ERR</param>
        void RespostaComandoERR(String comando)
        {
            RespostaComando("[" + comando + " ERR]");
        }
        
        /// <summary>
        ///  Função para enviar uma resposta que indica que a junta especificada em um comando que trate juntas não
        ///  foi reconhecida.
        /// </summary>
        void RespostaJNT_NAK()
        {
            RespostaComando("[JNT NAK]");
        }

        /// <summary>
        /// Função para guardar (commitar) os valores dos tempos dos servos antes de serem alterados.
        /// Usada como confirmação de que os valores foram setados corretamente na placa de controle
        /// dos servos. Estes valores podem ser recuperados pela função RollbackTempos().
        /// </summary>
        void CommitTempos()
        {
            servosTemp[0].tempoPulsoAlvo = servos[0].tempoPulsoAlvo;
            servosTemp[1].tempoPulsoAlvo = servos[1].tempoPulsoAlvo;
            servosTemp[2].tempoPulsoAlvo = servos[2].tempoPulsoAlvo;
            servosTemp[3].tempoPulsoAlvo = servos[3].tempoPulsoAlvo;
            servosTemp[4].tempoPulsoAlvo = servos[4].tempoPulsoAlvo;
            servosTemp[5].tempoPulsoAlvo = servos[5].tempoPulsoAlvo;
        }

        /// <summary>
        /// Função para recuperar os tempos anteriores dos servos em caso de erro.
        /// Os valores recuperados são os mesmos gravados pela função CommitTempos().
        /// </summary>
        void RollbackTempos()
        {
            servos[0].tempoPulsoAlvo = servosTemp[0].tempoPulsoAlvo;
            servos[1].tempoPulsoAlvo = servosTemp[1].tempoPulsoAlvo;
            servos[2].tempoPulsoAlvo = servosTemp[2].tempoPulsoAlvo;
            servos[3].tempoPulsoAlvo = servosTemp[3].tempoPulsoAlvo;
            servos[4].tempoPulsoAlvo = servosTemp[4].tempoPulsoAlvo;
            servos[5].tempoPulsoAlvo = servosTemp[5].tempoPulsoAlvo;
        }

        /// <summary>
        /// Função para guardar (commitar) os valores das velocidades dos servos antes de serem alteradas.
        /// Usada como confirmação de que os valores foram setados corretamente na placa de controle
        /// dos servos. Estes valores podem ser recuperados pela função RollbackVelocidades().
        /// </summary>
        void CommitVelocidades()
        {
            servosTemp[0].velTmpPulso = servos[0].velTmpPulso;
            servosTemp[1].velTmpPulso = servos[1].velTmpPulso;
            servosTemp[2].velTmpPulso = servos[2].velTmpPulso;
            servosTemp[3].velTmpPulso = servos[3].velTmpPulso;
            servosTemp[4].velTmpPulso = servos[4].velTmpPulso;
            servosTemp[5].velTmpPulso = servos[5].velTmpPulso;
        }

        /// <summary>
        /// Função para recuperar as velocidades anteriores dos servos em caso de erro.
        /// Os valores recuperados são os mesmos gravados pela função CommitVelocidades().
        /// </summary>
        void RollbackVelocidades()
        {
            servos[0].velTmpPulso = servosTemp[0].velTmpPulso;
            servos[1].velTmpPulso = servosTemp[1].velTmpPulso;
            servos[2].velTmpPulso = servosTemp[2].velTmpPulso;
            servos[3].velTmpPulso = servosTemp[3].velTmpPulso;
            servos[4].velTmpPulso = servosTemp[4].velTmpPulso;
            servos[5].velTmpPulso = servosTemp[5].velTmpPulso;
        }

        /// <summary>
        /// Função para guardar (commitar) os valores das acelerações dos servos antes de serem alteradas.
        /// Usada como confirmação de que os valores foram setados corretamente na placa de controle
        /// dos servos. Estes valores podem ser recuperados pela função RollbackAceleracoes().
        /// </summary>
        void CommitAceleracoes()
        {
            servosTemp[0].acelTmpPulso = servos[0].acelTmpPulso;
            servosTemp[1].acelTmpPulso = servos[1].acelTmpPulso;
            servosTemp[2].acelTmpPulso = servos[2].acelTmpPulso;
            servosTemp[3].acelTmpPulso = servos[3].acelTmpPulso;
            servosTemp[4].acelTmpPulso = servos[4].acelTmpPulso;
            servosTemp[5].acelTmpPulso = servos[5].acelTmpPulso;
        }

        /// <summary>
        /// Função para recuperar as velocidades anteriores dos servos em caso de erro.
        /// Os valores recuperados são os mesmos gravados pela função CommitAceleracoes().
        /// </summary>
        void RollbackAceleracoes()
        {
            servos[0].acelTmpPulso = servosTemp[0].acelTmpPulso;
            servos[1].acelTmpPulso = servosTemp[1].acelTmpPulso;
            servos[2].acelTmpPulso = servosTemp[2].acelTmpPulso;
            servos[3].acelTmpPulso = servosTemp[3].acelTmpPulso;
            servos[4].acelTmpPulso = servosTemp[4].acelTmpPulso;
            servos[5].acelTmpPulso = servosTemp[5].acelTmpPulso;
        }

        /// <summary>
        /// Envia para a UART string descrevendo o status de um servo, tais como tempo de pulso corrente, 
        /// tempo máximo, tempo mínimo etc.
        /// </summary>
        /// <param name="srv">servo cujos dados serão exibidos</param>
        void EnviaStatusServo(Servomotor srv)
        {
            String s;

            serialCom.WriteLine(srv.nome+":");

            serialCom.WriteLine(" TMP PULSO = " + srv.tempoPulsoAlvo.ToString("0000"));

            s = " TMP ";
            if (srv.sigla.Equals("GR"))
            {
                s += "GARRA FECHADA";
            }
            else
            {
                s += "MINIMO";
            }
            s += ": ";
            s += srv.tempoPulsoMin.ToString("0000");

            serialCom.WriteLine(s);

            s = " TMP ";
            if (srv.sigla.Equals("GR"))
            {
                s += "GARRA ABERTA";
            }
            else
            {
                s += "MAXIMO";
            }
            s += ": ";
            s += srv.tempoPulsoMax.ToString("0000");
            serialCom.WriteLine(s);

            s = " TMP ";
            if (srv.sigla.Equals("GR"))
            {
                s += "GARRA SEMI ABERTA";
            }
            else
            {
                s += "90 GRAUS";
            }
            s += ": ";
            s += srv.tempoPulsoPosNeutra.ToString("0000");
            serialCom.WriteLine(s);
        }

        /// <summary>
        /// Envia para a porta serial o status dos LEDS do braço robô
        /// </summary>
        void EnviaStatusLeds()
        {
            String s;
            int i;

            s = "LEDS: ";
            for (i = 0; i < 4; i++)
            {
                s += "P" + i.ToString() + ":";
                s += (bufferLeds[i]) ? " 1 " : " 0 ";
                s += "| ";
            }
            serialCom.WriteLine(s);

            s = "      ";
            for (i = 4; i < 8; i++)
            {
                s += "P" + i.ToString() + ":";
                s += (bufferLeds[i]) ? " 1 " : " 0 ";
                s += "| ";
            }
            serialCom.WriteLine(s);
        }

        /// <summary>
        /// Inicia o funcionamento e as configurações dos servos.
        /// </summary>
        void IniciarServos()
        {

            const float SERVO0_TEMPOMIN = 480;
            const float SERVO1_TEMPOMIN = 768;
            const float SERVO2_TEMPOMIN = 800;
            const float SERVO3_TEMPOMIN = 528;
            const float SERVO4_TEMPOMIN = 512;
            const float SERVO5_TEMPOMIN = 416;

            const float SERVO0_TEMPOMAX = 2432;
            const float SERVO1_TEMPOMAX = 2256;
            const float SERVO2_TEMPOMAX = 2208;
            const float SERVO3_TEMPOMAX = 2480;
            const float SERVO4_TEMPOMAX = 2432;
            const float SERVO5_TEMPOMAX = 2000;

            const float SERVO0_TMPNEUTRA = 1405;
            const float SERVO1_TMPNEUTRA = 1798;
            const float SERVO2_TMPNEUTRA = 2208;
            const float SERVO3_TMPNEUTRA = 1251;
            const float SERVO4_TMPNEUTRA = 1472;
            const float SERVO5_TMPNEUTRA = (SERVO5_TEMPOMAX + SERVO5_TEMPOMIN) / 2;

            const float SERVO0_TEMPOREP = SERVO0_TEMPOMIN;
            const float SERVO1_TEMPOREP = SERVO1_TEMPOMAX;
            const float SERVO2_TEMPOREP = 959;
            const float SERVO3_TEMPOREP = 1836;
            const float SERVO4_TEMPOREP = 1472;
            const float SERVO5_TEMPOREP = SERVO5_TEMPOMAX;

            const UInt16 SERVO0_VEL = 128;
            const UInt16 SERVO1_VEL = 128;
            const UInt16 SERVO2_VEL = 92;
            const UInt16 SERVO3_VEL = 92;
            const UInt16 SERVO4_VEL = 128;
            const UInt16 SERVO5_VEL = 100;

            const UInt16 SERVO0_ACEL = 16;
            const UInt16 SERVO1_ACEL = 16;
            const UInt16 SERVO2_ACEL = 24;
            const UInt16 SERVO3_ACEL = 32;
            const UInt16 SERVO4_ACEL = 32;
            const UInt16 SERVO5_ACEL = 0;

            //TMR0L = TMR0L_VAL;
            //TMR0H = TMR0H_VAL;
            //T0CON = T0CON_VAL;
    
            //INTCONbits.TMR0IE = 1; // Habilita interrupção do TIMER0

            //RST = 1; // Habilita (retira do reset) a placa de controle dos servos

            //delay_ms(1000); // Aguarda 1 segundo para a Mini Maestro terminar de iniciar
   
            servos[0].tempoPulsoAlvo = TempoPulsoInicial(0);
            servos[1].tempoPulsoAlvo = TempoPulsoInicial(1);
            servos[2].tempoPulsoAlvo = TempoPulsoInicial(2);
            servos[3].tempoPulsoAlvo = TempoPulsoInicial(3);
            servos[4].tempoPulsoAlvo = TempoPulsoInicial(4);
            servos[5].tempoPulsoAlvo = TempoPulsoInicial(5);
    
            servos[0].tempoPulsoMax = SERVO0_TEMPOMAX;
            servos[1].tempoPulsoMax = SERVO1_TEMPOMAX;
            servos[2].tempoPulsoMax = SERVO2_TEMPOMAX;
            servos[3].tempoPulsoMax = SERVO3_TEMPOMAX;
            servos[4].tempoPulsoMax = SERVO4_TEMPOMAX;

            servos[0].tempoPulsoMin = SERVO0_TEMPOMIN;
            servos[1].tempoPulsoMin = SERVO1_TEMPOMIN;
            servos[2].tempoPulsoMin = SERVO2_TEMPOMIN;
            servos[3].tempoPulsoMin = SERVO3_TEMPOMIN;
            servos[4].tempoPulsoMin = SERVO4_TEMPOMIN;

            servos[0].tempoPulsoPosNeutra = SERVO0_TMPNEUTRA;
            servos[1].tempoPulsoPosNeutra = SERVO1_TMPNEUTRA;
            servos[2].tempoPulsoPosNeutra = SERVO2_TMPNEUTRA;
            servos[3].tempoPulsoPosNeutra = SERVO3_TMPNEUTRA;
            servos[4].tempoPulsoPosNeutra = SERVO4_TMPNEUTRA;

            servos[5].tempoPulsoMax = SERVO5_TEMPOMAX; // Garra aberta
            servos[5].tempoPulsoMin = SERVO5_TEMPOMIN; // Garra fechada
            servos[5].tempoPulsoPosNeutra = SERVO5_TMPNEUTRA; // Garra semi-aberta

            servos[0].tempoPulsoRepouso = SERVO0_TEMPOREP;
            servos[1].tempoPulsoRepouso = SERVO1_TEMPOREP;
            servos[2].tempoPulsoRepouso = SERVO2_TEMPOREP;
            servos[3].tempoPulsoRepouso = SERVO3_TEMPOREP;
            servos[4].tempoPulsoRepouso = SERVO4_TEMPOREP;
            servos[5].tempoPulsoRepouso = SERVO5_TEMPOREP;

            servos[0].velTmpPulso = SERVO0_VEL;
            servos[1].velTmpPulso = SERVO1_VEL;
            servos[2].velTmpPulso = SERVO2_VEL;
            servos[3].velTmpPulso = SERVO3_VEL;
            servos[4].velTmpPulso = SERVO4_VEL;
            servos[5].velTmpPulso = SERVO5_VEL;

            servos[0].acelTmpPulso = SERVO0_ACEL;
            servos[1].acelTmpPulso = SERVO1_ACEL;
            servos[2].acelTmpPulso = SERVO2_ACEL;
            servos[3].acelTmpPulso = SERVO3_ACEL;
            servos[4].acelTmpPulso = SERVO4_ACEL;
            servos[5].acelTmpPulso = SERVO5_ACEL;

            for (int i = 0; i < 6; i++)
            {
                placaMiniMaestro24.canais[i].speed = servos[i].velTmpPulso;
                placaMiniMaestro24.canais[i].acceleration = servos[i].acelTmpPulso;
                placaMiniMaestro24.canais[i].max = (ushort)(servos[i].tempoPulsoMax * 4);
                placaMiniMaestro24.canais[i].min = (ushort)(servos[i].tempoPulsoMin * 4);
                // Este cálculo deve ser feito apenas aqui, pois mesmo que os limites sejam mudados, o ângulo de incremento
                // deve permanecer o mesmo.
                if (servos[i].proporcaoInversaTmpPulsoParaAngulo)
                {
                    servos[i].coefAngular = (servos[i].anguloMin - servos[i].anguloMax) / (servos[i].tempoPulsoMax - servos[i].tempoPulsoMin);
                    servos[i].offsetAngular = servos[i].anguloMin - servos[i].tempoPulsoMax * servos[i].coefAngular;
                }
                else
                {
                    servos[i].coefAngular = (servos[i].anguloMax - servos[i].anguloMin) / (servos[i].tempoPulsoMax - servos[i].tempoPulsoMin);
                    servos[i].offsetAngular = servos[i].anguloMax - servos[i].tempoPulsoMax * servos[i].coefAngular;
                }
                
                servos[i].conversaoTempoParaAnguloAtiva = true;
            }


            for(int i = 0; i < 5; i++)
            {
                servos[i].nome = "JUNTA "+ i.ToString("0");
                servos[i].sigla = "J" + i.ToString("0");                
                servos[i].idJST = ((char)(i+65)).ToString();
            }

            servos[5].nome = "GARRA";
            servos[5].sigla = "GR";
            servos[5].idJST = "G";
    
            CommitTempos(); // Para guardar os tempos antes do primeiro envio para a placa de controle
        }

        /// <summary>
        /// Função para fazer o braço robô assumir a posição de repouso. Deve ser usada principalmente
        /// na inicialização (ou reinicialização), para evitar movimentos bruscos dos servos.
        /// </summary>
        void PosicaoRepouso()
        {
            byte i;

            switch (this.estadoPosicaoRepouso)
            {
                case 0: // Início
                    frsTemp = feedbackRastrServos;
                    feedbackRastrServos = TipoFeedbackServos.fdbTemposDosServos; // Inicialização de posição ocorrerá com feedback

                    /* Reduz velocidade e aceleração para todas as juntas */
                    for (i = 0; i < 6; i++)
                    {
                        placaMiniMaestro24.SetAcceleration(i, 2);
                        placaMiniMaestro24.SetSpeed(i,10);
                    }

                    this.estadoPosicaoRepouso = 2;

                    break;

                case 1: // Estado final

                    break;

                case 2:
                    /* Gira a junta 0 para preparar o braço para assumir a posição de repouso */
                    servos[0].tempoPulsoAlvo = servos[0].tempoPulsoRepouso;
                    placaMiniMaestro24.SetTarget(0, servos[0].tempoPulsoAlvo);
                    ultCmdMovServos = ComandoProto.cmdIN1;
                    strUltCmdMovServos = "IN1";
                    tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpApenasUmServo;
                    canalCmd1ServoNaoBloq = 0;
                    this.estadoPosicaoRepouso = 3;
                    timer.Interval = 2000;
                    timer.Start();
                    break;

                case 3: // Espera concluir feedback dos servos
                    if (tipoCmdMovServosPorQt == TipoMultiplicidadeServos.mtpNenhumServo && !timer.Enabled)
                    {
                        this.estadoPosicaoRepouso = 4;
                    }
                    break;

                case 4:
                    /* Coloca o braço na posição de repouso */
                    for (i = 0; i < 6; i++)
                    {
                        servos[i].tempoPulsoAlvo = servos[i].tempoPulsoRepouso;
                        vetorTargets[i] = servos[i].tempoPulsoAlvo;
                    }
                    // Juntas 1 a 4 e garra
                    placaMiniMaestro24.SetMultipleTargets(5, 1, vetorTargets);
                    ultCmdMovServos = ComandoProto.cmdIN2;
                    strUltCmdMovServos = "IN2";
                    tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpVariosServos;
                    this.estadoPosicaoRepouso = 5;
                    timer.Interval = 2000;
                    timer.Start();
                    break;

                case 5: // Espera concluir feedback dos servos
                    if (tipoCmdMovServosPorQt == TipoMultiplicidadeServos.mtpNenhumServo && !timer.Enabled)
                    {
                        this.estadoPosicaoRepouso = 6;
                    }
                    break;

                

                case 6:
                    // Seta as velocidades e acelerações para os seus valores configurados
                    for (i = 0; i < 6; i++)
                    {
                        placaMiniMaestro24.SetAcceleration(i, servos[i].acelTmpPulso);
                        placaMiniMaestro24.SetSpeed(i, servos[i].velTmpPulso);
                    }

                    feedbackRastrServos = frsTemp;
                    serialCom.WriteLine("[PRONTO]");
                    this.estadoPosicaoRepouso = 1; // vai para o estado final
                    break;

            }            
        }

        /// <summary>
        /// Tenta obter da placa Mini Maestro os targets correntes dos servos. Função utilizada
        /// na inicialização dos servos.
        /// </summary>
        /// <param name="canal">Canal do servo na Mini Maestro</param>
        /// <returns>O valor corrente do target do servo, se houve comunicação bem sucedida com a Mini Maestro.
        /// Valor zero, se ocorreu algum erro na comunicação com a Mini Maestro, ou se a mesma estiver
        /// desconectada.</returns>
        float TempoPulsoInicial(byte canal)
        {
            float auxGetPosition;
            auxGetPosition = placaMiniMaestro24.GetPosition(canal);
            if(auxGetPosition == 0xFFFF)
            {
                RespostaComandoERR("INIT_CANAL_"+canal.ToString("00"));
                return 0;
            }
            return auxGetPosition;
        }

        /// <summary>
        /// Configura algum parâmetro de operação do servo (tempos máximo, mínimo ou central, velocidade, aceleracao etc), 
        /// dependendo de qual comando chamar esta função.
        /// Se o comando chamado não incluir o valor a ser setado como parâmetro, esta função trata, apenas, de exibir o valor setado.
        /// A variável indexServo é setada neste comando caso sua execução seja bem sucedida.
        /// </summary>
        /// <param name="cmdQueSeta">Endereço da string contendo o comando que setou o valor (TMX, TMN, T90, VEL, ACL).</param>
        /// <returns>true se o comando for corretamente executado, false caso contrário</returns>
        bool ConfigParamServo(String cmdQueSeta)
        {
            bool result = false;

            if(bufferProto[4] == 'J') // J0 a J4
            {
                if(bufferProto[5] >= '0' && bufferProto[5] <= '4')
                {
                    indexServo = (ushort)(bufferProto[5] - '0');
                }
                else
                {
                    RespostaJNT_NAK();
                    return false;
                }
            }
            else if (bufferProto[4] == 'G' && bufferProto[5] == 'R')
            {
                indexServo = 5;
            }
            else
            {
                RespostaComandoNAK(cmdQueSeta);
                return false;
            }

            if( cmdQueSeta.Equals("TMX") )
            {                
                result = VALSET(ref servos[indexServo].tempoPulsoMax);
            }
            else if(cmdQueSeta.Equals("TMN"))
            {
                result = VALSET(ref servos[indexServo].tempoPulsoMin);
            }
            else if(cmdQueSeta.Equals("T90"))
            {
                result = VALSET(ref servos[indexServo].tempoPulsoPosNeutra);
            }
            else if (cmdQueSeta.Equals("TRP"))
            {
                result = VALSET(ref servos[indexServo].tempoPulsoRepouso);
            }
            else if(cmdQueSeta.Equals("VEL"))
            {
                float valor = servos[indexServo].velTmpPulso;
                result = VALSET(ref valor);
                servos[indexServo].velTmpPulso = (ushort)valor;
            }
            else if(cmdQueSeta.Equals("ACL"))
            {
                float valor = servos[indexServo].acelTmpPulso;
                result = VALSET(ref valor);
                servos[indexServo].acelTmpPulso = (ushort)valor;
            }
            return result;
        }
        
        /// <summary>
        /// Função que trata de setar um valor de configuração de um servo e/ou de obter
        /// o valor dessa configuração e enviar para a UART 1 como resposta.
        /// </summary>
        /// <param name="valorASetar">referência para a variável a ser setada</param>
        /// <returns> true caso o comando (e os parâmetros do mesmo) sejam reconhecidos, false caso contrário</returns>
        bool VALSET(ref float valorASetar)
        {
            int i;
            float valorObtido;
            char milhar, centena, dezena, unidade;
            String strValorSet;
            bool flagErro = false;
            /*
             * [TMX<JX><TEMPO_4_DIGITOS>]
             * [TMXGR<TEMPO_4_DIGITOS>]
             * [TMX<JX>]
             * [TMXGR]
             * <JX>: junta X, onde X = 0,1,2,3,4
             * GR: garra
             */
            if (bufferProto.Length == 11){
                milhar = bufferProto[6];
                centena = bufferProto[7];
                dezena = bufferProto[8];
                unidade = bufferProto[9];
                if(Char.IsDigit(milhar) && Char.IsDigit(centena) && Char.IsDigit(dezena) && Char.IsDigit(unidade)){
                    valorObtido = 1000 * (int)(milhar - '0') + 100 * (int)(centena - '0') + 10 * (int)(dezena - '0') + (int)(unidade - '0');
                    if (valorObtido == 0)
                        valorASetar = 1;
                    else
                        valorASetar = valorObtido;
                }
                else
                    flagErro = true;
        
            }
            else if (bufferProto.Length != 7)
                flagErro = true;

            if(flagErro)
            {                
                RespostaComandoNAK(bufferProto.Substring(1,3));
                return false;
            }

            strValorSet = valorASetar.ToString("0000");

            bufferProto = "[" + bufferProto.Substring(1, 5) + "0000]";

            char[] bufferProtoCh = bufferProto.ToCharArray();
            
            for (i = 0; i < 4; i++)
            {
                bufferProtoCh[i + 6] = strValorSet.ToCharArray(i, 1)[0];
            }

            bufferProto = new String(bufferProtoCh);

            return true;
        }

        /// <summary>
        /// Função que seta o tempo de pulso do servo na estrutura que guarda os tempos dos servos. Caso o comando JST não venha
        /// com parâmetros, esta função apenas trata de montar a resposta do comando para que seja enviado pela UART os valores
        /// dos tempos de pulso do servo. Caso o JST venha apenas com 1 junta como parâmetro (sem o tempo), este comando monta a resposta
        /// do comando JST para enviar apenas o tempo de pulso corrente da junta especificada.
        /// </summary>
        /// <param name="ordemServo">valor inteiro (a partir de zero) que representa em que "ordem" o servo foi especificado no comando, para
        /// que a função possa encontrar a posição correta no buffer em que se encontra o valor a ser setado.</param>
        /// <param name="indiceServo">índice do servo no vetor de servos.</param>
        /// <returns>true caso os parâmetros do comando sejam reconhecidos, false caso contrário</returns>
        bool JST(int ordemServo, int indiceServo)
        {
            // ordemServo = 0, 1, 2, 3, 4, 5 (garra)
            float valor;
            int index, tamBuffer;
            char milhar, centena, dezena, unidade;
            int i;
            String strValorSet = "";

            index = ordemServo * 5;
            tamBuffer = bufferProto.Length;
            /*
             * [JSTA0010B0010C0040D0056E0041G0050]
             * [JSTC0040]
             * [JSTG0090B0031]
             * [JST]
             * [JSTA]
             * [JSTB]
             * [JSTG]
             */
            // tamanho máximo = 35 - tamanhos 5 6 10 15 20 25 30 35
            if(tamBuffer >= 10  &&  tamBuffer % 5 == 0) // Aqui o valor do tempo é setado, se for o caso
            {
                milhar = bufferProto[5+index];
                centena = bufferProto[6+index];
                dezena = bufferProto[7+index];
                unidade = bufferProto[8+index];
                if (Char.IsDigit(milhar) && Char.IsDigit(centena) && Char.IsDigit(dezena) && Char.IsDigit(unidade))
                {
                    valor = 1000 * (int)(milhar - '0') + 100 * (int)(centena - '0') + 10 * (int)(dezena - '0') + (int)(unidade - '0');
                    if (valor != 0)
                    {
                        if (valor > servos[indiceServo].tempoPulsoMax) valor = servos[indiceServo].tempoPulsoMax;
                        if (valor < servos[indiceServo].tempoPulsoMin) valor = servos[indiceServo].tempoPulsoMin;
                    }
                    servos[indiceServo].tempoPulsoAlvo = valor;
                }
                else
                {
                    return false;
                }
            }

            strValorSet = servos[indiceServo].tempoPulsoAlvo.ToString("0000");
            resposta += servos[indiceServo].idJST+strValorSet;

            return true;
        }
   
        /// <summary>
        /// Envia para a porta serial o feedback de um comando
        /// </summary>
        /// <param name="feedback"></param>
        void EnviarFeedback(String feedback)
        {
            serialCom.WriteLine("");
            serialCom.Write(feedback);
        }
        /// <summary>
        /// Reseta as variáveis de controle de feedback dos comandos de acionamento
        /// </summary>
        void ResetaVariaveisDeFeedBack()
        {
            tipoCmdMovServosPorQt = 0;
            ultCmdMovServos = ComandoProto.cmdNone;
            canalCmd1ServoNaoBloq = CANAL_INDEFINIDO;
        }

        /// <summary>
        /// Método que realiza a parada total propriamente dita
        /// </summary>
        void ParadaTotal()
        {
            float posicao;
    
            for(byte canal = 0; canal < 6; canal++)
            {
                posicao = placaMiniMaestro24.GetPosition(canal);
                placaMiniMaestro24.SetTarget(canal, posicao);
            }
    
        }

        /// <summary>
        /// Método para retomar as velocidades nos canais dos servos na Mini Maestro 24
        /// </summary>
        void RetomarVelocidadesAposParadaTotal1Servo()
        {
            if (paradaTotalSolicitada)
            {
                paradaTotalSolicitada = false;
                for (byte canal = 0; canal < 6; canal++)
                {
                    placaMiniMaestro24.SetSpeed(canal, servos[canal].velTmpPulso);
                }
            }
        }

        /// <summary>
        /// Função responsável por enviar as respostas dos comandos dos servos de forma não bloqueante.
        /// Ou seja, quaisquer novos comandos podem ser recebidos durante o envio da resposta.
        /// Esta função pode ser usada de forma bloqueante se ela for executada dentro de um loop que
        /// teste se tipoCmdMovServosPorQt for diferente de mtpApenasUmServo.
        /// </summary>
        private void RespEFeedbackMovServos()
        {
            byte result;
            UInt16 posicao;
    
            switch(tipoCmdMovServosPorQt)
            {
                case TipoMultiplicidadeServos.mtpApenasUmServo: // Acionamento de 1 servo apenas
                {
                    result = placaMiniMaestro24.GetMovingState();
                    if (result == 0x01) // Em movimento
                    {
                        switch(this.feedbackRastrServos)
                        {
                        case TipoFeedbackServos.fdbSemFeedback: // sem feedback
                            if (paradaTotalSolicitada)
                                ParadaTotal();
                            break;
                        case TipoFeedbackServos.fdbTemposDosServos: // Tempos dos servos
                            if (paradaTotalSolicitada)
                                ParadaTotal();
                            posicao = (ushort)(placaMiniMaestro24.GetPosition(canalCmd1ServoNaoBloq) / 4);
                            EnviarFeedback("[MOV" + servos[canalCmd1ServoNaoBloq].sigla + posicao.ToString("0000") + "]");
                            break;
                        case TipoFeedbackServos.fdbSinalDeMovimento: // Sinal de movimento
                            if (paradaTotalSolicitada)
                                ParadaTotal();
                            EnviarFeedback("[MOV]");
                            break;
                        }
                    }
                    else if (result == 0x00) // Parado
                    {
                        String s;

                        CommitTempos();

                        RetomarVelocidadesAposParadaTotal1Servo();

                        s = "["+strUltCmdMovServos;
                        
                        if(this.ultCmdMovServos == ComandoProto.cmdCTZ || this.ultCmdMovServos == ComandoProto.cmdIN1)
                        {
                            s += servos[canalCmd1ServoNaoBloq].sigla;
                        }
                        posicao = (ushort)(placaMiniMaestro24.GetPosition(canalCmd1ServoNaoBloq) / 4);

                        s += posicao.ToString("0000") + "]";

                        RespostaComando(s);

                        ResetaVariaveisDeFeedBack();
                    }
                    else
                    {
                        RollbackTempos();
                        RetomarVelocidadesAposParadaTotal1Servo();
                        RespostaComandoERR(strUltCmdMovServos);
                        if (result != 0xFF)
                        {
                            serialCom.WriteLine("");
                            serialCom.WriteLine("[ERR"+result.ToString("X2")+"]");
                        }

                        ResetaVariaveisDeFeedBack();
                    }
                    break;
                }
        
                case TipoMultiplicidadeServos.mtpVariosServos: // Acionamento de vários servos
                {
                    byte canal;

                    result = placaMiniMaestro24.GetMovingState();
                    
                    if (result == 0x01) // Em movimento
                    {
                        switch(this.feedbackRastrServos)
                        {
                        case TipoFeedbackServos.fdbSemFeedback: // Sem feedback
                            if (paradaTotalSolicitada)
                                ParadaTotal();
                            break;

                        case TipoFeedbackServos.fdbTemposDosServos: // Feedback com valores de posição (tempos) dos servos
                            string s = String.Empty;
                            
                            s += "[MOV";
                            for (canal = 0; canal < 6; canal++)
                            {
                                posicao = (ushort)(placaMiniMaestro24.GetPosition(canal) / 4);
                                if (paradaTotalSolicitada)
                                    placaMiniMaestro24.SetTarget(canal, posicao);
                                s += servos[canal].idJST;
                                s += posicao.ToString("0000");
                            }
                            s += "]";
                            EnviarFeedback(s);
                            break;

                        case TipoFeedbackServos.fdbSinalDeMovimento: // feedback aviso de movimento apenas
                            if (paradaTotalSolicitada)
                                ParadaTotal();
                            EnviarFeedback("[MOV]");
                            break;
                        }
                    }
                    else if (result == 0x00) // Parado
                    {
                        string s = String.Empty;
                        CommitTempos();
                        RetomarVelocidadesAposParadaTotal1Servo();
                        s += "["+strUltCmdMovServos;
                        for (canal = 0; canal < 6; canal++)
                        {
                            posicao = (ushort)(placaMiniMaestro24.GetPosition(canal) / 4);
                            s += servos[canal].idJST;
                            s += posicao.ToString("0000");
                        }
                        s += "]";
                        RespostaComando(s);

                        ResetaVariaveisDeFeedBack();
                    }
                    else
                    {
                        RollbackTempos();
                        RetomarVelocidadesAposParadaTotal1Servo();
                        RespostaComandoERR(strUltCmdMovServos);
                        if (result != 0xFF)
                        {
                            serialCom.WriteLine("");
                            serialCom.WriteLine("[ERR" + result.ToString("XX") + "]");
                        }

                        ResetaVariaveisDeFeedBack();
                    }
                    break;
                }
        
                default:
                    break;
            }
        }

        /// <summary>
        /// Função responsável pela decodificação dos comandos, acionamento geral e configurações.
        /// </summary>
        private void DecodificaProtocolo()
        {
            ushort i;
            short idxSrv;
    
            /*
             * [LED11111111]
             * [LED10101010]
             * [LED00000000]
             * [LED]
            RESPOSTA: O mesmo comando aplicado, se especificado os bits, ou o comando com os bits no estado corrente

            Acende ou apaga os leds do PORTLEDS
             */
            if (Comando("LED")) 
            {
                if (bufferProto.Length == 13) // SETA OS LEDS
                {
                    char c;
                    
                    for (i = 0; i < 8; i++)
                    {
                        c = bufferProto[i + 4];
                        if (c == '0' || c == '1') {
                            bufferLeds[7-i] = (c == '1');
                        } else {
                            bufferLeds[7-i] = false;                            
                            bufferProto.Replace(c, '0');
                        }
                    }

                    OnComandoLEDRecebido(EventArgs.Empty);
                    
                    RespostaComando(bufferProto);
                }
                else if (bufferProto.Length == 5) // DEVOLVE O STATUS DOS LEDS
                {
                    char[] bufferProtoCh;

                    bufferProtoCh = new char[13];
                    bufferProtoCh[0] = '[';
                    bufferProtoCh[1] = 'L';
                    bufferProtoCh[2] = 'E';
                    bufferProtoCh[3] = 'D';
                    bufferProtoCh[12] = ']';
                    
                    for (i = 0; i < 8; i++)
                    {
                        if (bufferLeds[7-i])
                        {
                            bufferProtoCh[i + 4] = '1';
                        }
                        else
                        {
                            bufferProtoCh[i + 4] = '0';
                        }
                    }

                    bufferProto = new String(bufferProtoCh);

                    RespostaComando(bufferProto);
                }
                else // COMANDO [LED] NÃO RECONHECIDO
                {
                    RespostaComandoNAK("LED");
                }
            }
            else if (Comando("PRT"))
            {
                if (placaMiniMaestro24.GetMovingState() == 0x01)
                {
                    paradaTotalSolicitada = true;
                    for (byte canal = 0; canal < 6; canal++)
                    {
                        placaMiniMaestro24.SetSpeed(canal, 1);
                    }
                }
                else
                {
                    RespostaComando("[PRT OK]");
                }
            }
            /*
             * [GA]
             * Garra Abrir = Abre a garra completamente
             * Resposta: [GAXXXX] onde XXXX = tempo de pulso do servo da garra (em us) ao estar completamente aberta.
             *           Se o feedback for tipo 1 (tempo dos servos): [MOVGRXXXX], onde XXXX = posição corrente do servo (em us)
             *           Se o feedback for tipo 2 (sinal de movimento): [MOV]
             *           Os feedbacks sempre virão antes da resposta [GAXXXX]. Se a garra já estiver completamente aberta,
             *           apenas a resposta [GAXXXX] será enviada.
             */
            else if (Comando("GA"))
            {
                OnComandoAcionamentoServoRecebido(EventArgs.Empty);

                this.comando = ComandoProto.cmdGA;

                this.garra.servoDaGarra.tempoPulsoAlvo = this.garra.servoDaGarra.tempoPulsoMax;

                placaMiniMaestro24.SetTarget(5, servos[5].tempoPulsoAlvo);

                if (this.tipoCmdMovServosPorQt < TipoMultiplicidadeServos.mtpVariosServos)
                {
                    if (canalCmd1ServoNaoBloq != 5 && canalCmd1ServoNaoBloq != CANAL_INDEFINIDO)
                    {
                        tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpVariosServos;
                        ultCmdMovServos = ComandoProto.cmdJST;
                        strUltCmdMovServos = "JST";
                        canalCmd1ServoNaoBloq = CANAL_INDEFINIDO;
                    }
                    else
                    {
                        this.ultCmdMovServos = ComandoProto.cmdGA;
                        this.strUltCmdMovServos = "GA";
                        this.tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpApenasUmServo;
                        this.canalCmd1ServoNaoBloq = 5;
                    }
                }
            }
            /*
             * [GF]
             * Garra Fechar = Fecha a garra completamente
             * * Resposta: [GFXXXX] onde XXXX = tempo de pulso do servo da garra (em us) ao estar completamente fechada.
             *           Se o feedback for tipo 1 (tempo dos servos): [MOVGRXXXX], onde XXXX = posição corrente do servo (em us)
             *           Se o feedback for tipo 2 (sinal de movimento): [MOV]
             *           Os feedbacks sempre virão antes da resposta [GFXXXX]. Se a garra já estiver completamente fechada,
             *           apenas a resposta [GFXXXX] será enviada.
             */
            else if (Comando("GF"))
            {
                OnComandoAcionamentoServoRecebido(EventArgs.Empty);

                this.comando = ComandoProto.cmdGF;

                this.garra.servoDaGarra.tempoPulsoAlvo = this.garra.servoDaGarra.tempoPulsoMin;

                placaMiniMaestro24.SetTarget(5, servos[5].tempoPulsoAlvo);

                if (this.tipoCmdMovServosPorQt < TipoMultiplicidadeServos.mtpVariosServos)
                {
                    if (canalCmd1ServoNaoBloq != 5 && canalCmd1ServoNaoBloq != CANAL_INDEFINIDO)
                    {
                        tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpVariosServos;
                        ultCmdMovServos = ComandoProto.cmdJST;
                        strUltCmdMovServos = "JST";
                        canalCmd1ServoNaoBloq = CANAL_INDEFINIDO;
                    }
                    else
                    {
                        this.ultCmdMovServos = ComandoProto.cmdGF;
                        this.strUltCmdMovServos = "GF";
                        this.tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpApenasUmServo;
                        this.canalCmd1ServoNaoBloq = 5;
                    }
                }
            }

            /*
             * [CTZ<JX>]
             * [CTZGR]
             *
             * Sendo <JX> = J0,...,J4
             *        GR = Garra
             * Centraliza em 90 graus a junta correspondente ou deixa a garra semiaberta
             * Resposta: [CTZ<JX>XXXX]
             *           [CTZGRXXXX]
             *           onde XXXX é o tempo de pulso corrente do servo da junta (ou da garra) em us.
             *           Se o feedback for do tipo 1 (tempo dos servos):
             *             [MOV<JX>XXXX]
             *             [MOVGRXXXX]
             *           Se o feedback for do tipo 2(sinal de movimento): [MOV]
             *
             */
            else if (Comando("CTZ"))
            {
                if (bufferProto.Length != 7)
                {
                    RespostaComandoNAK("CTZ");
                }
                if (bufferProto[4] == 'J')
                {
                    if (bufferProto[5] >= '0' && bufferProto[5] <= '4')
                    {
                        byte canal;

                        OnComandoAcionamentoServoRecebido(EventArgs.Empty);

                        canal = Byte.Parse(bufferProto[5].ToString());
                        servos[canal].tempoPulsoAlvo = servos[canal].tempoPulsoPosNeutra;

                        placaMiniMaestro24.SetTarget(canal, servos[canal].tempoPulsoAlvo);

                        if (this.tipoCmdMovServosPorQt < TipoMultiplicidadeServos.mtpVariosServos)
                        {
                            if (canalCmd1ServoNaoBloq != canal && canalCmd1ServoNaoBloq != CANAL_INDEFINIDO)
                            {
                                tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpVariosServos;
                                ultCmdMovServos = ComandoProto.cmdJST;
                                strUltCmdMovServos = "JST";
                                canalCmd1ServoNaoBloq = CANAL_INDEFINIDO;
                            }
                            else
                            {
                                ultCmdMovServos = ComandoProto.cmdCTZ;
                                strUltCmdMovServos = "CTZ";
                                tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpApenasUmServo;
                                canalCmd1ServoNaoBloq = canal;
                            }
                        }
                    }
                    else
                    {
                        RespostaJNT_NAK();
                    }
                }
                else if (bufferProto[4] == 'G' && bufferProto[5] == 'R')
                {
                    OnComandoAcionamentoServoRecebido(EventArgs.Empty);

                    servos[5].tempoPulsoAlvo = servos[5].tempoPulsoPosNeutra;

                    placaMiniMaestro24.SetTarget(5, servos[5].tempoPulsoAlvo);

                    if (this.tipoCmdMovServosPorQt < TipoMultiplicidadeServos.mtpVariosServos)
                    {
                        if (canalCmd1ServoNaoBloq != 5 && canalCmd1ServoNaoBloq != CANAL_INDEFINIDO)
                        {
                            tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpVariosServos;
                            ultCmdMovServos = ComandoProto.cmdJST;
                            strUltCmdMovServos = "JST";
                            canalCmd1ServoNaoBloq = CANAL_INDEFINIDO;
                        }
                        else
                        {
                            ultCmdMovServos = ComandoProto.cmdCTZ;
                            strUltCmdMovServos = "CTZ";
                            tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpApenasUmServo;
                            canalCmd1ServoNaoBloq = 5;
                        }
                    }
                }
                else
                {
                    RespostaJNT_NAK();
                }
            }

            /*
             * [JSTA0010B0010C0040D0056E0041G0050]
             * [JSTC0040]
             * [JSTG0090B0031]
             * [JST]
             * [JSTA]
             * [JSTB]
             * [JSTG]
             *
             * RESPOSTA: O mesmo comando, com os valores de tempo das juntas especificadas realmente setados, se for aplicado com as juntas e o tempo.
             *           Os tempos de todas as juntas, se for aplicado apenas o [JST]
             *           O tempo da junta especificada, se for aplicado [JST<J>], onde <J> = A,...,E,G
             * onde:
             * JST: Comando de Junta Setar Tempo (Tempo de pulso dos servos das juntas e de abertura/fechamento da garra são setados dos valores
             *       especificados no pacote)
             * A, B, C, D, E: Representam as juntas. Na resposta, o tempo estará sempre entre o tempo máximo e o tempo mínimo configurados.
             * G: Representa a garra (abertura). Na resposta, o tempo estará sempre entre o tempo máximo e o tempo mínimo configurados para a garra.
             * Se for aplicado o comando JST com pelo menos 1 junta e um tempo de pulso:
             *    - Se o feedback for do tipo 1 (tempos dos servos): [MOVAXXXXBXXXXCXXXXDXXXXEXXXXGXXXX]
             *      onde XXXX é tempo do servo corrente correspondente
             *    - Se o feedback for do tipo 2 (sinal de movimento): [MOV]
             *    Se todos os servos especificados no comando JST (com junta e tempo de pulso) já estiverem
             *    nas posições especificadas (ou em suas posições limites, de modos que não podem se mover além delas)
             *    apenas a resposta [JSTAXXXXBXXXXCXXXXDXXXXEXXXXGXXXX] (com os valores de tempo em us no lugar dos XXXX) será enviada.
             */
            else if (Comando("JST"))
            {
                ushort iServo;
                int tamBuffer;

                tamBuffer = bufferProto.Length; // tamanho máximo = 35 - tamanhos 5 6 10 15 20 25 30 35
                if (tamBuffer > 35 || tamBuffer < 5 || (tamBuffer % 5 != 0 && tamBuffer != 6))
                {
                    RespostaComandoNAK("JST");
                }
                else if (tamBuffer == 5) //[JST]
                {
                    resposta = "[JST";
                    for (i = 0; i < 6; i++)
                    {
                        /* Ao chamar o [JST] sem nenhum parâmetro, nenhum valor será setado nas variáveis */
                        JST(i, i);
                    }
                    resposta += "]";

                    RespostaComando(resposta);
                }
                else if (tamBuffer == 6) // [JST<J>], onde <J> = A,...,E,G
                {
                    /* Ao chamar o JST com uma junta (sem o valor do tempo) será mostrado o tempo correspondente da junta especificada,
                       sem que nenhum valor seja setado */
                    if (bufferProto[4] >= 'A' && bufferProto[4] <= 'E')
                    {
                        idxSrv = (short)(bufferProto[4] - 'A');
                        resposta = "[JST";
                        JST(0, idxSrv);
                        resposta += ']';
                        RespostaComando(resposta);
                    }
                    else if (bufferProto[4] == 'G')
                    {
                        resposta = "[JST";
                        JST(0, 5);
                        resposta += ']';
                        RespostaComando(resposta);
                    }
                    else
                    {
                        RespostaJNT_NAK();
                    }
                }
                else // JST com as juntas e os valores de tempo
                {
                    byte primeiroCanal;
                    i = 0;
                    iServo = 4;
                    primeiroCanal = 5;
                    resposta = "[JST";
                    while (i < 6 && iServo < tamBuffer - 1)
                    {
                        if (bufferProto[iServo] >= 'A' && bufferProto[iServo] <= 'E')
                        {
                            idxSrv = (short)(bufferProto[iServo] - 'A');
                            if (primeiroCanal > (byte)idxSrv) primeiroCanal = (byte)idxSrv;
                            if (!JST(i, idxSrv)) i = 6;
                        }
                        else if (bufferProto[iServo] == 'G')
                        {
                            if (!JST(i, 5)) i = 6;
                        }
                        else
                        {
                            i = 6;
                        }

                        i++;
                        iServo = (ushort)(4 + 5 * i);
                    }
                    resposta += "]";

                    if (i > 6)
                    {
                        RespostaComandoNAK("JST");
                    }
                    else
                    {
                        OnComandoAcionamentoServoRecebido(EventArgs.Empty);

                        float[] targets = new float[6];
                        for (i = 0; i < targets.Length; i++)
                        {
                            targets[i] = servos[i].tempoPulsoAlvo;
                        }

                        // Transmissão dos tempos para a placa Mini Maestro.
                        placaMiniMaestro24.SetMultipleTargets((byte)(6 - primeiroCanal), primeiroCanal, targets);

                        ultCmdMovServos = ComandoProto.cmdJST;
                        strUltCmdMovServos = "JST";
                        tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpVariosServos;
                        canalCmd1ServoNaoBloq = CANAL_INDEFINIDO;
                    }
                }
            }
            /*
             * [RPS]
             * Resposta: [RPSAXXXXBXXXXCXXXXDXXXXEXXXX], onde XXXX é o valor em microssegundos da posição
             * dos servos
             *
             * Coloca o braço robô na posição de repouso configurada com o comando TRP. Notar que a garra não é manipulada neste caso.
             * - Se o feedback for do tipo 1 (tempos dos servos): [MOVAXXXXBXXXXCXXXXDXXXXEXXXXGXXXX]
             *   onde XXXX é tempo do servo corrente correspondente
             * - Se o feedback for do tipo 2 (sinal de movimento): [MOV]
             * Se todos os servos estiverem na posição de repouso,
             * apenas a resposta [JSTAXXXXBXXXXCXXXXDXXXXEXXXXGXXXX] (com os valores de tempo em us no lugar dos XXXX) será enviada.
             */
            else if (Comando("RPS"))
            {
                if (bufferProto.Length == 5)
                {
                    OnComandoAcionamentoServoRecebido(EventArgs.Empty);

                    float[] targets = new float[6];
                    for (i = 0; i < 5; i++)
                    {
                        servos[i].tempoPulsoAlvo = servos[i].tempoPulsoRepouso;
                        targets[i] = servos[i].tempoPulsoAlvo;
                    }

                    // Transmissão dos tempos para a placa Mini Maestro.
                    placaMiniMaestro24.SetMultipleTargets(5, 0, targets);

                    ultCmdMovServos = ComandoProto.cmdRPS;
                    strUltCmdMovServos = "RPS";
                    tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpVariosServos;
                    canalCmd1ServoNaoBloq = CANAL_INDEFINIDO;
                }
                else
                {
                    RespostaComandoNAK("RPS");
                }
            }
            /*
                * [DSL]
                * Resposta: [DSLA0000B0000C0000D0000E0000]
                *
                * Desliga as juntas e a garra do braço robô, de forma que não haja controle
                * de posicionamento nas mesmas.
                */
            else if (Comando("DSL"))
            {
                if (bufferProto.Length == 5)
                {
                    OnComandoAcionamentoServoRecebido(EventArgs.Empty);

                    float[] targets = new float[6];
                    for (i = 0; i < 5; i++)
                    {
                        servos[i].tempoPulsoAlvo = servos[i].tempoPulsoRepouso;
                        targets[i] = 0;
                    }

                    // Transmissão dos tempos para a placa Mini Maestro.
                    placaMiniMaestro24.SetMultipleTargets(5, 0, targets);

                    ultCmdMovServos = ComandoProto.cmdDSL;
                    strUltCmdMovServos = "DSL";
                    tipoCmdMovServosPorQt = TipoMultiplicidadeServos.mtpVariosServos;
                    canalCmd1ServoNaoBloq = CANAL_INDEFINIDO;
                }
                else
                {
                    RespostaComandoNAK("DSL");
                }
            }
            /*
             * [TMX<JX><TEMPO_4_DIGITOS>]
             * [TMXGR<TEMPO_4_DIGITOS>]
             * [TMX<JX>]
             * [TMXGR]
             * Onde:
             * <JX>: junta X, onde X = 0,1,2,3,4
             * GR: garra
             * <TEMPO_4_DIGITOS>: tempo de pulso para o servomotor em us
             *
             * RESPOSTA: [TMX<JX><TEMPO_4_DIGITOS_SETADO>]
             *           [TMXGR<TEMPO_4_DIGITOS_SETADO>]
             * onde <TEMPO_4_DIGITOS_SETADO>: Tempo máximo setado (não ultrapassa
             *                                o limite máximo de 50% do periodo da onda).
             *
             */
            else if (Comando("TMX"))
            {
                if (ConfigParamServo("TMX"))
                {
                    RespostaComando(bufferProto);
                }
            }

            /*
             * [TMN<JX><TEMPO_4_DIGITOS>]
             * [TMNGR<TEMPO_4_DIGITOS>]
             * [TMN<JX>]
             * [TMNGR]
             * <JX>: junta X, onde X = 0,1,2,3,4
             * GR: garra
             * <TEMPO_4_DIGITOS>: tempo de pulso para o servomotor em us
             *
             * RESPOSTA: [TMN<JX><TEMPO_4_DIGITOS_SETADO>]
             *           [TMNGR<TEMPO_4_DIGITOS_SETADO>]
             * onde <TEMPO_4_DIGITOS_SETADO>: Tempo mínimo realmente setado
             */
            else if (Comando("TMN"))
            {
                if (ConfigParamServo("TMN"))
                {
                    RespostaComando(bufferProto);
                }
            }

            /*
             * [T90<JX><TEMPO_4_DIGITOS>]
             * [T90GR<TEMPO_4_DIGITOS>]
             * [T90<JX>]
             * [T90GR]
             * <JX>: junta X, onde X = 0,1,2,3,4
             * GR: garra
             * <TEMPO_4_DIGITOS>: tempo de pulso para o servomotor em us
             *
             * RESPOSTA: [TMN<JX><TEMPO_4_DIGITOS_SETADO>]
             *           [TMNGR<TEMPO_4_DIGITOS_SETADO>]
             * onde <TEMPO_4_DIGITOS_SETADO>: Tempo do servo realmente setado quando estiver na posição considerada central
             */
            else if (Comando("T90"))
            {
                if (ConfigParamServo("T90"))
                {
                    RespostaComando(bufferProto);
                }

            }

            /*
             * [TRP<JX><TEMPO_4_DIGITOS>]
             * [TRPGR<TEMPO_4_DIGITOS>]
             * [TRP<JX>]
             * [TRPGR]
             * <JX>: junta X, onde X = 0,1,2,3,4
             * GR: garra
             * <TEMPO_4_DIGITOS>: tempo de pulso para o servomotor em us
             *
             * RESPOSTA: [TRP<JX><TEMPO_4_DIGITOS_SETADO>]
             *           [TMNGR<TEMPO_4_DIGITOS_SETADO>]
             * onde <TEMPO_4_DIGITOS_SETADO>: Tempo do servo realmente setado considerado como a posição de repouso
             */
            else if (Comando("TRP"))
            {
                if (ConfigParamServo("TRP"))
                {
                    RespostaComando(bufferProto);
                }
            }

            /*
             * [VEL<JX><VEL_4_DIGITOS>]
             * [VELGR<VEL_4_DIGITOS>]
             * [VEL<JX>]
             * [VELGR]
             * <JX>: junta X, onde X = 0,1,2,3,4
             * GR: garra
             * <VEL_4_DIGITOS>: velocidade de variação do tempo de pulso para o servomotor
             *
             * RESPOSTA: [VEL<JX><VEL_4_DIGITOS_SETADA>]
             *           [TMNGR<VEL_4_DIGITOS_SETADA>]
             * onde <VEL_4_DIGITOS_SETADO>: Valor de velocidade de variação do tempo de pulso do servo (4 dígitos)
             */
            else if (Comando("VEL"))
            {
                if (ConfigParamServo("VEL"))
                {
                    if (placaMiniMaestro24.SetSpeed((byte)indexServo, servos[indexServo].velTmpPulso))
                    {
                        CommitVelocidades();
                        RespostaComando(bufferProto);
                    }
                    else
                    {
                        RollbackVelocidades();
                        RespostaComandoERR("VEL");
                    }
                }
            }

            /*
             * [ACL<JX><ACEL_4_DIGITOS>]
             * [ACLGR<ACEL_4_DIGITOS>]
             * [ACL<JX>]
             * [ACLGR]
             * <JX>: junta X, onde X = 0,1,2,3,4
             * GR: garra
             * <ACEL_4_DIGITOS>: aceleração de variação do tempo de pulso para o servomotor
             *
             * RESPOSTA: [VEL<JX><ACEL_4_DIGITOS_SETADA>]
             *           [TMNGR<ACEL_4_DIGITOS_SETADA>]
             * onde <ACEL_4_DIGITOS_SETADO>: Valor de aceleração do tempo de pulso do servo
             */
            else if (Comando("ACL"))
            {
                if (ConfigParamServo("ACL"))
                {
                    if (placaMiniMaestro24.SetAcceleration((byte)indexServo, servos[indexServo].acelTmpPulso))
                    {
                        CommitAceleracoes();
                        RespostaComando(bufferProto);
                    }
                    else
                    {
                        RollbackAceleracoes();
                        RespostaComandoERR("ACL");
                    }
                }
            }

            /*
             * [FRS<OP>]
             * [FRS]
             * Onde <OP> = 0, 1 ou 2, sendo que:
             *        0 = sem feedback
             *        1 = valores de posição (tempos) dos servos enquanto em movimento
             *        2 = sinal de movimentação dos servos
             * Resposta: o mesmo comando com o mesmo valor de <OP> setado.
             *
             * Feedback de Rastreamento dos Servos
             * Configuração para qual tipo de feedback os comandos JST, CTZ, GA e GF enviarão
             * a UART 1 durante o movimento dos servos.
             */
            else if (Comando("FRS"))
            {
                int tam;
                tam = bufferProto.Length;

                if (tam == 6) // Seta o tipo do feedback
                {
                    byte b;
                    b = (byte)(bufferProto[4] - '0');

                    switch (b)
                    {
                        case 0: // Sem Feedback
                        case 1: // Posição dos servos
                        case 2: // sinal de movimentação
                            feedbackRastrServos = (TipoFeedbackServos)b;
                            RespostaComando(bufferProto);
                            break;
                        default:
                            RespostaComandoNAK("FRS");
                            break;
                    }
                }
                else if (tam == 5) // Responde com o tipo do feedback corrente
                {

                    RespostaComando("[FRS" + ((int)feedbackRastrServos).ToString() + "]");
                }
                else
                {
                    RespostaComandoNAK("FRS");
                }
            }

            /*
             * [CSB<B>]
             * [CSB]
             * Onde <B> = 0, 1
             * Resposta: o mesmo comando com o mesmo valor de <B> setado.
             *
             * Comando Servos Bloqueantes
             * Configuração para permitir ou não que os comandos JST, CTZ, GA e GF permitam a recepção
             * de novos comandos pela UART 1 enquanto os servos estiverem se movimentando.
             */
            else if (Comando("CSB"))
            {
                int tam;
                tam = bufferProto.Length;

                if (tam == 6)
                {
                    byte op;
                    op = (byte)(bufferProto[4] - '0');

                    switch (op)
                    {
                        case 0:
                        case 1:
                            comandosServosBloqueantes = (op == 1);
                            RespostaComando(bufferProto);
                            break;
                        default:
                            RespostaComandoNAK("CSB");
                            break;
                    }
                }
                else if (tam == 5)
                {
                    int b = (comandosServosBloqueantes) ? 1 : 0;
                    RespostaComando("[CSB" + b.ToString() + "]");
                }
                else
                {
                    RespostaComandoNAK("CSB");
                }
            }

            /*
             * [EMM]
             * Erros da Mini Maestro
             * RESPOSTA: [EMM<valor_em_hexa_4_digitos>]
             *
             * onde <valor_em_hexa_4_digitos>: Valor correspondente ao erro ocorrido na mini maestro.
             *                                 FFFF significa que a Mini Maestro está desconectada
             *                                 enquanto 0000 significa ausência de erros
             */
            else if (Comando("EMM"))
            {
                ushort erros;

                erros = placaMiniMaestro24.GetErrors();

                RespostaComando("[EMM" + erros.ToString("X4") + "]");
            }


            /*
             * [GTP<JX>]
             * [GTPGR]
             * [GTP<JX>S]
             * [GTPGRS]
             * Onde:
             * GTP: comando Get PoSition da placa Mini Maestro
             * <JX>: junta X, onde X = 0,1,2,3,4
             * GR: garra
             * S: Usa-se o S no final do comando para sinalizar para a Ready For PIC que o valor
             * obtido da placa Mini Maestro deve ser atualizado no campo de tempo de pulso do
             * servo correspondente.
             * Resposta:
             * [GTP<JX><VALOR_4_DIGITOS>]
             * [GTPGR<VALOR_4_DIGITOS>]
             * [GTP<JX><VALOR_4_DIGITOS>S]
             * [GTPGR<VALOR_4_DIGITOS>S]
             *
             * Onde:
             * <VALOR_4_DIGITOS>: Valor do tempo de pulso (Target) obtido da placa Mini Maestro
             * S: Para indicar que o valor do Target foi sincronizado com o tempo de pulso do servo
             *    registrado na estrutura do servo correspondente.
             */
            else if (Comando("GTP"))
            {
                float position;
                char sincroniza;
                string s = String.Empty;

                if (bufferProto.Length < 7 || bufferProto.Length > 8 || (bufferProto.Length == 8 && bufferProto[6] != 'S'))
                {
                    RespostaComandoNAK("GTP");
                }
                else if (bufferProto[4] == 'J')
                {
                    if (bufferProto[5] >= '0' && bufferProto[5] <= '4')
                    {
                        byte canal;
                        canal = (byte)(bufferProto[5] - '0');

                        position = placaMiniMaestro24.GetPosition(canal);

                        if (position != 0xFFFF)
                        {
                            sincroniza = bufferProto[6];

                            s += "[GTP" + bufferProto.Substring(4, 2) + position.ToString("0000");
                            if (sincroniza == 'S')
                            {
                                servos[canal].tempoPulsoAlvo = position / 4;
                                s += "S]";
                                CommitTempos();

                            }
                            else
                            {
                                s += ']';
                            }

                            RespostaComando(s);
                        }
                        else
                        {
                            RespostaComandoERR("GTP");
                        }
                    }
                    else
                    {
                        RespostaJNT_NAK();
                    }
                }
                else if (bufferProto[4] == 'G' && bufferProto[5] == 'R')
                {
                    position = placaMiniMaestro24.GetPosition(5);

                    if (position != 0xFFFF)
                    {
                        sincroniza = bufferProto[6];

                        s += "[GTP" + bufferProto.Substring(4, 2) + position.ToString("0000");
                        if (sincroniza == 'S')
                        {
                            servos[5].tempoPulsoAlvo = position;
                            CommitTempos();
                            s += "S]";
                        }
                        else
                        {
                            s += "]";
                        }

                        RespostaComando(bufferProto);
                    }
                    else
                    {
                        RespostaComandoERR("GTP");
                    }
                }
                else
                {
                    RespostaJNT_NAK();
                }
            }

            /*
             * [GMS]
             * Comando Get Moving State da placa Mini Maestro
             * RESPOSTA: [GMS1]  quando está havendo movimento de algum servo
             *           [GMS0]  quando não há movimento de nenhum servo
             *           [GMS ERR] quando a placa está desconectada
             */
            else if (Comando("GMS"))
            {
                byte ms;

                ms = placaMiniMaestro24.GetMovingState();

                switch (ms)
                {
                    case 0x00:
                        RespostaComando("[GMS0]");
                        break;
                    case 0x01:
                        RespostaComando("[GMS1]");
                        break;
                    default:
                        RespostaComandoERR("GMS");
                        break;
                }
            }

            /*
             * [STT]
             * STatus de Todos os dispositivos
             */
            else if (Comando("STT"))
            {
                serialCom.WriteLine("");
                serialCom.WriteLine("TEMPO DOS SERVOS:");

                for (i = 0; i < 6; i++)
                {
                    EnviaStatusServo(servos[i]);
                }
                EnviaStatusLeds();
            }

            /*
             *
             * [RST]
             * ReSeT em todos os hardwares conectados à Ready For PIC
             *
             * [RSTM]
             * Reset apenas na placa Mini Maestro 24
             */
            else if (Comando("RST"))
            {
                if (bufferProto.Length == 6 && bufferProto[4] == 'M')
                {
                    serialCom.WriteLine("");
                    serialCom.WriteLine("[RESETANDO MINI MAESTRO 24]");
                    IniciarServos(); // Retira a Mini Maestro do reset e reinicia os servos.
                    serialCom.WriteLine("");
                    serialCom.WriteLine("[PRONTO]");
                }
                else if (bufferProto.Length == 5)
                {
                    RespostaComando(bufferProto);
                    serialCom.WriteLine("[RESETANDO]");
                    timer.Interval = 5000;
                    timer.Start();

                    IniciarServos();
                    this.estadoPosicaoRepouso = 0;

                    // TODO: Comando RST: Avaliar se é possível simular o acendimento dos leds que é realizado por este comando na placa Ready for PIC

                    //PORTLEDS = 0;
                    //for (i = 0; i < 8; i++) {
                    //    PORTLEDS = PORTLEDS << 1 | 0x01;
                    //    delay_ms(200);
                    //}

                    //k = 1;
                    //for (i = 254; i >= 0; i -= k) {
                    //    PORTLEDS = PORTLEDS & i;
                    //    k *= 2;
                    //    delay_ms(200);
                    //}

                    //RESET(); // Reset do PIC18F45K22. Os demais hardwares serão resetados durante a reinicialização.
                }
                else
                {
                    RespostaComandoNAK("RST");
                }
            }
            /*
             * [ECH<B>]
             * [ECH]
             * Onde <B> = 0, 1
             * Resposta: o mesmo comando com o mesmo valor de <B> setado.
             *
             * Eco CHar
             * Configuração para habilitar (<B> = 1) ou desabilitar (<B> = 0) o echo
             * dos caracteres recebidos pela UART1.
             */
            else if (Comando("ECH"))
            {
                int tam;
                tam = bufferProto.Length;

                if (tam == 6)
                {
                    byte op;
                    op = (byte)(bufferProto[4] - '0');

                    switch (op)
                    {
                        case 0:
                        case 1:
                            ecoCaracteresAtivado = (op == 1);
                            RespostaComando(bufferProto);
                            break;
                        default:
                            RespostaComandoNAK("ECH");
                            break;
                    }
                }
                else if (tam == 5)
                {
                    int b = (ecoCaracteresAtivado) ? 1 : 0;
                    RespostaComando("[ECH" + b.ToString() + "]");
                }
                else
                {
                    RespostaComandoNAK("ECH");
                }
            }
            /*
             * 
             * Quando comando não é reconhecido, cai neste else
             * 
             */
            else
            {
                RespostaComando("[CMD NAK]");
            }
        }
        

        /// <summary>
        /// Método não implementado
        /// </summary>
        /// <returns></returns>
        public override float MomentoInercia()
        {
            throw new System.NotImplementedException();
        }
    }
}
