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
using Microsoft.Xna.Framework;

namespace BracoRobo.Classes
{
    /// <summary>
    /// Um tipo delegate para conectar notificações de mudanças de posição no eixo do servomotor.
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="e"></param>
    public delegate void ChangedEventHandler(object sender, EventArgs e);

    /// <summary>
    /// Representa um servomotor. Um servomotor é um motor que rotaciona seu eixo para um determinado ângulo e procura mantê-lo, aplicando
    /// torque quando houver algum torque contrário sendo aplicado no seu eixo.
    /// </summary>
    public class Servomotor : Motor
    {
        /// <summary>
        /// Evento que irá notificar qualquer mudança em anguloCorrente
        /// </summary>
        public event ChangedEventHandler ChangedAnguloCorrente;
        /// <summary>
        /// Invoca o evento ChangedAnguloCorrente. Chamado sempre que AnguloCorrente mudar de valor
        /// </summary>
        /// <param name="e"></param>
        protected virtual void OnChanged(EventArgs e)
        {
            if (ChangedAnguloCorrente != null)
            {
                ChangedAnguloCorrente(this, e);
            }
        }

        /**** Atributos do servomotor ****/
        /* Características a serem obtidas no datasheet do motor (ou na loja) */
        /// <summary>
        /// Máxima voltagem aceita pelo servomotor (em Volts). Pode ser obtido no datasheet do motor ou na loja onde o mesmo for comprado.
        /// </summary>
        public float voltagemMax;
        /// <summary>
        /// Mínima voltagem aceita pelo servomotor (em Volts). Pode ser obtido no datasheet do motor ou na loja onde o mesmo for comprado.
        /// </summary>
        public float voltagemMin;
        /// <summary>
        /// Velocidade máxima do eixo de rotação do servomotor (em segundos/60º, sem carga) ao aplicar a voltagem contida em voltagemMin.
        /// A unidade segundos/60º significa quantos segundos leva para o servomotor girar um ângulo de 60º. Pode ser obtido no datasheet do motor ou na loja onde o mesmo for comprado.
        /// </summary>
        public float velocidadeMax;
        /// <summary>
        /// Velocidade mínima do eixo de rotação do servomotor (em segundos/60º, sem carga) ao aplicar a voltagem contida em voltagemMin
        /// A unidade segundos/60º significa quantos segundos leva para o servomotor girar um ângulo de 60º. Pode ser obtido no datasheet do motor ou na loja onde o mesmo for comprado.
        /// </summary>
        public float velocidadeMin;
        /// <summary>
        /// Torque máximo (em Kgf.cm) que o motor é capaz de aplicar sobre uma carga ao aplicar a voltagem contida em voltagemMax.
        /// Pode ser obtido no datasheet do motor ou na loja onde o mesmo for comprado.
        /// </summary>
        public float torqueMax;
        /// <summary>
        /// Torque mínimo (em Kgf.cm) que o motor é capaz de aplicar sobre uma carga ao aplicar a voltagem contida em voltagemMin.
        /// Pode ser obtido no datasheet do motor ou na loja onde o mesmo for comprado.
        /// </summary>
        public float torqueMin;
        /// <summary>
        /// Ângulo máximo que o eixo do servo assume. Pode ser obtido no datasheet do motor ou na loja onde o mesmo for comprado.
        /// Também pode ser especificado ângulos menores que o ângulo máximo do servo para limitar mais a faixa de movimento.
        /// Notar que este ângulo não necessariamente irá corresponder aos ângulos utilizados nas matrizes dos modelos 3D. Neste
        /// caso, é necessária uma conversão de ângulos.
        /// </summary>
        public float anguloMax;
        /// <summary>
        /// Ângulo mínimo que o servo assume. Pode ser obtido no datasheet do motor ou na loja onde o mesmo for comprado.
        /// Também pode ser especificado valores maiores que o ângulo mínimo do servo para limitar mais a faixa de movimento.
        /// Notar que este ângulo não necessariamente irá corresponder aos ângulos utilizados nas matrizes dos modelos 3D. Neste
        /// caso, é necessária uma conversão de ângulos.
        /// </summary>
        public float anguloMin;
        /// <summary>
        /// Ângulo alvo que se quer que o eixo do servo assuma.
        /// Notar que este ângulo não necessariamente irá corresponder aos ângulo utilizado nas matrizes do modelo 3D. Neste
        /// caso, é necessária uma conversão de ângulos.
        /// </summary>
        public float anguloAlvo;
        /// <summary>
        /// Ângulo corrente que o eixo do servo assume.
        /// Notar que este ângulo não necessariamente irá corresponder aos ângulos utilizado nas matrizes do modelo 3D. Neste
        /// caso, é necessária uma conversão de ângulos.
        /// </summary>
        private float anguloCorrente;
        /// <summary>
        /// Ângulo corrente que o eixo do servo assume.
        /// Notar que este ângulo não necessariamente irá corresponder aos ângulos utilizado nas matrizes do modelo 3D. Neste
        /// caso, é necessária uma conversão de ângulos.
        /// </summary>
        public float AnguloCorrente
        {
            get { return anguloCorrente; }
            set
            {
                anguloCorrente = value;
                if (conversaoAnguloCorrParaAngGiroEixo)
                {
                    float tmpPulsoCorrentePrev;
                    this.AnguloGiroEixo = new Vector3(this.AnguloGiroEixo.X, this.anguloGiroEixoInicial.Y + this.sinalPropagacaoAnguloCorrente * this.AnguloCorrente, this.AnguloGiroEixo.Z);
                    tmpPulsoCorrentePrev = (float)Math.Round((AnguloCorrente - offsetAngular) / coefAngular); // Para prever o tempoPulsoCorrente, caso seja usado o teclado para mover o braço robô.
                    Console.WriteLine(this.sigla + ":" + AnguloCorrente.ToString() + "° TempoPulsoCorrente equivalente: "+tmpPulsoCorrentePrev.ToString()+"us");
                    //Console.WriteLine(this.AnguloGiroEixo.ToString());
                }
                OnChanged(EventArgs.Empty);
            }
        }
        /// <summary>
        /// Diferença entre a property AnguloCorrente e AnguloCorrenteDH
        /// </summary>
        public float difAnguloCorrenteDH = 0.0f;
        /// <summary>
        /// Ângulo corrente que o eixo do servo assume convertido para o valor correspondente em parâmetros de Denavit-Hatenberg
        /// </summary>
        public float AnguloCorrenteDH
        {
            get { return AnguloCorrente + difAnguloCorrenteDH; }
        }

        /// <summary>
        /// Coeficiente para conversão linear de tempo de pulso para graus. Usado quando há conversão de tempo de pulso
        /// para graus.
        /// </summary>
        public float coefAngular;
        /// <summary>
        /// Valor de offset para conversão de tempo de pulso para graus. Usado quando há conversão de tempo de pulso 
        /// para graus.
        /// </summary>
        public float offsetAngular;
        /// <summary>
        /// Guarda o sinal (positivo ou negativo) utilizado na conversão de tempo de pulso (us) para ângulo em graus. Usar 1.0f para positivo ou -1.0f para negativo.
        /// </summary>
        public float sinalPropagacaoAnguloCorrente = 1.0f;

        /* Atributos dinâmicos do servomotor */
        /// <summary>
        /// Voltagem aplicada sobre o motor (em Volts)
        /// </summary>
        public float voltagemAplic;

        /// <summary>
        /// Período de atualização (em microssegundos) do sinal PWM aplicado ao servomotor.
        /// É o tempo máximo entre um pulso e outro de controle do servomotor.
        /// </summary>
        public float periodoAtualizacaoPWM = 20;
        /// <summary>
        /// Duração (em microssegundos) do sinal do pulso corrente para o servomotor (atributo)
        /// </summary>
        private float tempoPulsoCorrente;
        /// <summary>
        /// Duração (em microssegundos) do sinal do pulso corrente para o servomotor (Property)
        /// </summary>
        public float TempoPulsoCorrente
        {
            get { return tempoPulsoCorrente; }
            set 
            { 
                tempoPulsoCorrente = value;
                if (conversaoTempoParaAnguloAtiva && tempoPulsoCorrente != 0)
                {
                    AnguloCorrente = coefAngular * tempoPulsoCorrente + offsetAngular;                 
                }
            }
        }
        /// <summary>
        /// Tempo (em microssegundos) do sinal do pulso alvo para o servomotor. Ao determinar uma nova posição para o eixo
        /// do servo, este é o atributo a ser setado.
        /// </summary>
        public float tempoPulsoAlvo;
        /// <summary>
        /// Duração (em microssegundos) do sinal de pulso para o servomotor girar para o ângulo máximo
        /// </summary>
        public float tempoPulsoMax;
        /// <summary>
        /// Duração (em microssegundos) do sinal de pulso para o servomotor girar para o ângulo mínimo
        /// </summary>
        public float tempoPulsoMin;
        /// <summary>
        /// Duração (em microssegundos) do sinal de pulso para o servomotor girar para o ângulo correspondente à posição central (neutra)
        /// </summary>
        public float tempoPulsoPosNeutra;
        /// <summary>
        /// Tempo de pulso para o servo assumir a posição de repouso. Útil para o caso de religamento ou reinicialização da placa de controle dos servos.
        /// </summary>
        public float tempoPulsoRepouso;
        /// <summary>
        /// Velocidade máxima de variação do tempo de pulso em (0,25us)/(10ms). Se zero, o servo trabalha na sua velocidade máxima.
        /// </summary>
        public UInt16 velTmpPulso;
        /// <summary>
        /// Aceleração da variação do tempo de pulso em (0,25us)/(10ms)/(80ms). Se zero, o servo trabalha com a sua máxima aceleração.
        /// </summary>
        public UInt16 acelTmpPulso;
        /// <summary>
        /// Número que representa a quantidade de posições diferentes que o servomotor pode assumir
        /// </summary>
        public int resolucao;

        /// <summary>
        /// Sigla da junta correspondente ao servomotor
        /// </summary>
        public string sigla;
        /// <summary>
        /// Caractere que corresponde à junta correspondente ao servo no comando JST
        /// </summary>
        public string idJST;
        /// <summary>
        /// Campo para o nome associado ao servo.
        /// </summary>
        public string nome;
        /// <summary>
        /// Canal da placa Mini Maestro ao qual o servomotor é conectado.
        /// </summary>
        public CanalMiniMaestro canalMM;
        /// <summary>
        /// Flag para indicar se deve haver ou não conversão de tempo de pulso para ângulo.
        /// </summary>
        public bool conversaoTempoParaAnguloAtiva = false;
        /// <summary>
        /// Flag para indicar que a conversão de tempo de pulso para graus segue uma proporção inversa (true), ou
        /// seja, se o tempo de pulso máximo corresponde ao angulo mínimo e o tempo de pulso mínimo corresponde
        /// ao tempo de pulso máximo.
        /// </summary>
        public bool proporcaoInversaTmpPulsoParaAngulo = false;
        /// <summary>
        /// Flag para indicar se haverá (true) ou não (false) conversão do atributo AnguloCorrente para anguloGiroEixo
        /// </summary>
        public bool conversaoAnguloCorrParaAngGiroEixo = true;

        /// <summary>
        /// Construtor padrão
        /// </summary>
        public Servomotor()
        {
           
        }

        /// <summary>
        /// Método para calcular a faixa de operação (em microssegundos) do pulso a ser aplicado no servomotor.
        /// Ou seja, é a diferença entre o maior e o menor pulso a ser aplicado no servomotor.
        /// </summary>
        /// <returns>Valor, em microssegundos, da faixa de operação</returns>
        public float FaixaDeOperacao()
        {
            return this.tempoPulsoMax - this.tempoPulsoMin;
        }
        /// <summary>
        /// Método para calcular a frequência PWM (em KHz) necessária para aplicar no servomotor
        /// </summary>
        /// <returns>Frequência PWM, em KHz, a ser aplicada no servomotor</returns>
        public float frequenciaPWM()
        {
            return 1.0f / (this.FaixaDeOperacao() / this.resolucao);
        }
        /// <summary>
        /// Método que atualiza a posição do servomotor. Será levado em consideração a velocidade
        /// e a aceleração do eixo do servo.
        /// </summary>
        public void Update(GameTime gameTime)
        {
            //double t = gameTime.ElapsedGameTime.TotalSeconds;

        }
    }
}
