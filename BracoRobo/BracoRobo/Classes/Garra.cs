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
using System;

namespace BracoRobo.Classes
{
    /// <summary>
    /// Classe para representar uma garra MK2
    /// </summary>
    public class GarraMK2 : ComponenteFisico
    {
        /* Componentes da garra */
        /// <summary>
        /// Motor para movimentar as engrenagens dos dedos da garra.
        /// </summary>
        public Servomotor servoDaGarra;
        /// <summary>
        /// Base da garra. Por ela ser o referencial da garra, ela será tratada, dentro da garra, como uma base fixa.
        /// </summary>
        public Base baseGarra;
        /// <summary>
        /// Peça da garra MK2 responsável por conectar o servo com os segmentos engrenados
        /// </summary>
        public EixoEngrenagemAmortecida eixoEngrenagemAmortecida;
        /// <summary>
        /// Segmento engrenado que se conecta ao dedo esquerdo e ao segmento engrenado direito
        /// </summary>
        public SegmentoComEngrenagem segmentoComEngrenagemEsquerdo;
        /// <summary>
        /// Segmento engrenado que se conecta ao dedo direito e ao segmento engrenado esquerdo
        /// </summary>
        public SegmentoComEngrenagem segmentoComEngrenagemDireito;
        /// <summary>
        /// Segmento que conecta o corpo da base da garra ao dedo esquerdo.
        /// </summary>
        public SegmentoArticulacaoGarra segmentoArticuladoEsquerdo;
        /// <summary>
        /// Segmento que conecta o corpo da base da garra ao dedo direito.
        /// </summary>
        public SegmentoArticulacaoGarra segmentoArticuladoDireito;
        /// <summary>
        /// Dedo esquerdo da garra.
        /// </summary>
        public DedoGarra dedoEsquerdo;
        /// <summary>
        /// Dedo direito da garra.
        /// </summary>
        public DedoGarra dedoDireito;
        
        
        /// <summary>
        /// Objeto que a garra está segurando. Se for null, significa que a garra não está segurando nada.
        /// </summary>
        public ComponenteFisico objeto = null;

        /// <summary>
        /// Construtor padrão. Inicia os componentes da garra com valores padrões.
        /// 
        /// </summary>
        public GarraMK2()
        {
            /* Base da garra */
            this.baseGarra = new Base();
            this.baseGarra.posReferencial = new Vector3(0.0f, 0.0f, 0.0f);
            this.baseGarra.centroGiro = this.baseGarra.posReferencial;
            this.baseGarra.anguloGiro = new Vector3(0.0f, 0.0f, 0.0f);
            //this.baseGarra.velocidadeAng = new Vector3(0.0f, 4.0f, 0.0f);

            /* Eixo de engrenagem amortecida */
            this.eixoEngrenagemAmortecida = new EixoEngrenagemAmortecida();
            this.eixoEngrenagemAmortecida.posReferencial = new Vector3(0.627394f, 3.226761f, -0.650517f);
            this.eixoEngrenagemAmortecida.centroGiro = new Vector3(-0.65f, 3.1f, -0.650517f);
            this.eixoEngrenagemAmortecida.anguloGiro = new Vector3(0.0f, 0.0f, 0.0f);
            //this.eixoEngrenagemAmortecida.anguloGiro = new Vector3(0.0f, 0.0f, 0.0f);

            /* Segmento com engrenagem esquerdo */
            this.segmentoComEngrenagemEsquerdo = new SegmentoComEngrenagem();
            this.segmentoComEngrenagemEsquerdo.anguloGiro = new Vector3(0.0f, 0.0f, 0.0f);
            this.segmentoComEngrenagemEsquerdo.velocidadeAng = -9/18 * this.eixoEngrenagemAmortecida.velocidadeAng;
            this.segmentoComEngrenagemEsquerdo.centroGiro = new Vector3(-1.35f, 5.0f, 0.0f);

            /* Segmento com engrenagem direito */
            this.segmentoComEngrenagemDireito = new SegmentoComEngrenagem();
            this.segmentoComEngrenagemDireito.anguloGiro = new Vector3(0.0f, 0.0f, 0.0f);
            this.segmentoComEngrenagemDireito.velocidadeAng = -this.segmentoComEngrenagemEsquerdo.velocidadeAng;
            this.segmentoComEngrenagemDireito.centroGiro = new Vector3(1.35f, 5.0f, 0.0f);

            /* Segmento articulado esquerdo */
            this.segmentoArticuladoEsquerdo = new SegmentoArticulacaoGarra();
            this.segmentoArticuladoEsquerdo.anguloGiro = this.segmentoComEngrenagemEsquerdo.anguloGiro;
            this.segmentoArticuladoEsquerdo.velocidadeAng = this.segmentoComEngrenagemEsquerdo.velocidadeAng;
            this.segmentoArticuladoEsquerdo.centroGiro = new Vector3(-0.5f, 7.0f, 0.0f);

            /* Segmento articulado direito */
            this.segmentoArticuladoDireito = new SegmentoArticulacaoGarra();
            this.segmentoArticuladoDireito.anguloGiro = this.segmentoComEngrenagemDireito.anguloGiro;
            this.segmentoArticuladoDireito.velocidadeAng = this.segmentoComEngrenagemDireito.velocidadeAng;
            this.segmentoArticuladoDireito.centroGiro = new Vector3(0.5f, 7.0f, 0.0f);

            /* Dedo esquerdo */
            this.dedoEsquerdo = new DedoGarra();
            //this.dedoEsquerdo.posDedoAberto = new Vector3(-3.901226f, 6.807381f, -0.050765f);
            //this.dedoEsquerdo.posDedoFechado = new Vector3(-0.801226f, 9.907381f, -0.050765f);
            this.dedoEsquerdo.posDedoAberto = new Vector3(0.0f, 0.0f, 0.0f);
            this.dedoEsquerdo.posDedoFechado = new Vector3(2.991812f, 3.098112f, 0.0f);
            this.dedoEsquerdo.posReferencial = this.dedoEsquerdo.posDedoAberto;
            //this.dedoEsquerdo.posReferencial = this.dedoEsquerdo.posDedoFechado;

            /* Dedo direito */
            this.dedoDireito = new DedoGarra();
            //this.dedoDireito.posDedoAberto = new Vector3(3.901226f, 6.807381f, -0.050765f);
            //this.dedoDireito.posDedoFechado = new Vector3(0.801226f, 9.907381f, -0.050765f);
            this.dedoDireito.posDedoAberto = new Vector3(0.0f, 0.0f, 0.0f);
            this.dedoDireito.posDedoFechado = new Vector3(-2.991812f, 3.098112f, 0.0f);
            this.dedoDireito.posReferencial = this.dedoDireito.posDedoAberto;

            /* Servo da garra */
            this.servoDaGarra = new Servomotor();
            this.servoDaGarra.conversaoAnguloCorrParaAngGiroEixo = false;
            this.servoDaGarra.ChangedAnguloCorrente += new ChangedEventHandler(servoDaGarra_ChangedAnguloCorrente);
            this.servoDaGarra.sigla = "GR";
            this.servoDaGarra.idJST = "G";
            // Posições dos componentes físicos em relação ao referencial local do servo
            //this.servoDaGarra.posReferencial = new Vector3(0.0f, -0.658824f, -0.583333f);
            //this.servoDaGarra.corpo.posReferencial = new Vector3(0.0f, -0.658824f, -0.583333f);
            //this.servoDaGarra.eixo.posReferencial = new Vector3(0.0f, 0.2f, 0.0f);
            //this.servoDaGarra.eixo.centroGiro = this.servoDaGarra.eixo.posReferencial;
            //this.servoDaGarra.posReferencial = new Vector3(-0.65f, 3.105f, 1.7f);
            this.servoDaGarra.posReferencial = new Vector3(-0.65f, 3.1f, 1.8f);
            this.servoDaGarra.corpo.posReferencial = this.servoDaGarra.posReferencial;
            this.servoDaGarra.eixo.posReferencial = this.servoDaGarra.posReferencial;
            this.servoDaGarra.anguloGiro = new Vector3(-90.0f, 0.0f, -90.0f);
            this.servoDaGarra.corpo.anguloGiro = this.servoDaGarra.anguloGiro;
            //this.servoDaGarra.eixo.anguloGiro = new Vector3(-90.0f, 0.0f, 0.0f);
            this.servoDaGarra.anguloGiroEixoInicial = new Vector3(-90.0f, 0.0f, 0.0f);
            this.servoDaGarra.AnguloGiroEixo = this.servoDaGarra.anguloGiroEixoInicial;
            this.servoDaGarra.centroEixoGiro = this.servoDaGarra.posReferencial + new Vector3(0.0f, 0.0f, 2.0f);
            this.servoDaGarra.velAngularEixoMax = new Vector3(0.0f, 0.0f, 1.0f);
            this.servoDaGarra.VelocidadeAngularEixo = new Vector3(0.0f, 0.0f, 0.0f);
            this.servoDaGarra.sinalPropagacaoAnguloCorrente = 1.0f;
            this.servoDaGarra.anguloMin = 0.0f;
            this.servoDaGarra.anguloMax = 180.0f;
            this.servoDaGarra.AnguloCorrente = 0.0f;
            this.servoDaGarra.FixarEm(this.baseGarra);
            this.servoDaGarra.AcoplarAoEixo(this.eixoEngrenagemAmortecida, EixoCartesiano.Zpos);
            this.servoDaGarra.proporcaoInversaTmpPulsoParaAngulo = true;

            // Centro de giro da garra (valor padrão)
            this.centroGiro = this.baseGarra.centroGiro;

        }

        /// <summary>
        /// Método não implementado
        /// </summary>
        /// <returns></returns>
        public override float MomentoInercia()
        {
            throw new System.NotImplementedException();
        }

        /// <summary>
        /// Atribui à propriedade "objeto" da garra um componente físico. Significa que, após a garra agarrar o objeto,
        /// quaisquer movimentos que a garra fizer (rotação e/ou translação), o objeto irá fazer igual.
        /// </summary>
        /// <param name="obj">Representa o objeto que será agarrado.</param>
        /// <returns>Retorna true, se agarrou o objeto. Retorna false, caso contrário.</returns>
        public bool AgarraObjeto(ComponenteFisico obj)
        {
            /* Lembrar de acrescentar outras validações antes de agarrar o objeto, tais como as dimensões do objeto (se compatível com a garra),
               peso do objeto (se haverá atrito suficiente entre o objeto e a garra para segurar o peso do objeto). */
            
            // Se a garra já está segurando um objeto, ela não poderá agarra outro.
            if (this.objeto != null)
                return false;
            this.objeto = obj;
            return true;
        }

        /// <summary>
        /// Atribui null à propriedade "objeto" da garra, significando que a garra soltou o objeto que ela estava agarrando,
        /// qualquer que seja.
        /// </summary>
        /// <returns>Retorna true, se o objeto foi solto. Retorna false, caso contrário.</returns>
        public bool SoltaObjeto()
        {
            // Se não há o que soltar.
            if (this.objeto == null)
                return false;
            // Atribuir null à propriedade objeto significa que a garra está soltando o objeto.
            this.objeto = null;
            return true;
        }

        /// <summary>
        /// Abre a garra. A cada execução deste método, os dedos da garra deverão se deslocar uma distância dependente da velocidade 
        /// de rotação do eixo do motor.
        /// </summary>
        public void Abre()
        {

            // Se dedo direito ainda não estiver em posição de aberto
            if (this.dedoDireito.posReferencial.X < this.dedoDireito.posDedoAberto.X && this.dedoDireito.posReferencial.Y > this.dedoDireito.posDedoAberto.Y)
            {
                // Velocidade angular do eixo do servo da garra
                this.servoDaGarra.VelocidadeAngularEixo = -this.servoDaGarra.velAngularEixoMax;

                // Velocidade angular da Engrenagem do servo -> Velocidade angular da Engrenagem do eixo engrenado esquerdo
                this.segmentoComEngrenagemEsquerdo.velocidadeAng = - this.eixoEngrenagemAmortecida.numDentes * (this.eixoEngrenagemAmortecida.velocidadeAng) / this.segmentoComEngrenagemEsquerdo.numDentes;
                // Velocidade angular do segmento articulado esquerdo
                this.segmentoArticuladoEsquerdo.velocidadeAng = this.segmentoComEngrenagemEsquerdo.velocidadeAng;

                // Velocidade angular da Engrenagem do eixo engrenado esquerdo -> Velocidade angular do eixo engrenado direito
                this.segmentoComEngrenagemDireito.velocidadeAng = -this.segmentoComEngrenagemEsquerdo.velocidadeAng;
                // Velocidade angular do segmento articulado direito
                this.segmentoArticuladoDireito.velocidadeAng = this.segmentoComEngrenagemDireito.velocidadeAng;

                // Ângulo do eixo do servo da garra. Lembrar que será atualizado, também, o ângulo de giro do componente físico 
                // que estiver acoplado ao eixo do servo (neste caso, a engrenagem amortecida, representada por eixoEngrenagemAmortecida)
                this.servoDaGarra.AnguloGiroEixo += this.servoDaGarra.VelocidadeAngularEixo;
                // Ângulo da Engrenagem do eixo engrenado esquerdo
                this.segmentoComEngrenagemEsquerdo.anguloGiro += this.segmentoComEngrenagemEsquerdo.velocidadeAng;
                // Ângulo do segmento articulado esquerdo
                this.segmentoArticuladoEsquerdo.anguloGiro += this.segmentoArticuladoEsquerdo.velocidadeAng;
                // Ângulo da Engrenagem do eixo engrenado direito
                this.segmentoComEngrenagemDireito.anguloGiro += this.segmentoComEngrenagemDireito.velocidadeAng;
                // Ângulo do segmento articulado direito
                this.segmentoArticuladoDireito.anguloGiro += this.segmentoArticuladoDireito.velocidadeAng;
                
                
                // Posição do dedo direito
                this.dedoDireito.posReferencial =
                    new Vector3(this.segmentoComEngrenagemDireito.tamanhoSegmento * ((float)(Math.Cos(MathHelper.ToRadians(this.segmentoComEngrenagemDireito.anguloGiro.Z))) - 1.0f),
                                this.segmentoComEngrenagemDireito.tamanhoSegmento * ((float)(Math.Sin(MathHelper.ToRadians(this.segmentoComEngrenagemDireito.anguloGiro.Z)))),
                                 0.0f);
            }
            // Corrige a posição do dedo direito caso ultrapasse a posição de "aberto"
            if (this.dedoDireito.posReferencial.X > this.dedoDireito.posDedoAberto.X || this.dedoDireito.posReferencial.Y < this.dedoDireito.posDedoAberto.Y)
            {
                this.dedoDireito.posReferencial = this.dedoDireito.posDedoAberto;
            }
            // Se dedo esquerdo ainda não estiver em posição de "aberto"
            if (this.dedoEsquerdo.posReferencial.X > this.dedoEsquerdo.posDedoAberto.X && this.dedoEsquerdo.posReferencial.Y > this.dedoEsquerdo.posDedoAberto.Y)
            {   
                // Posição do dedo esquerdo
                this.dedoEsquerdo.posReferencial =
                    new Vector3(this.segmentoComEngrenagemEsquerdo.tamanhoSegmento * (1.0f - (float)(Math.Cos(MathHelper.ToRadians(this.segmentoComEngrenagemEsquerdo.anguloGiro.Z)))),
                                 -this.segmentoComEngrenagemEsquerdo.tamanhoSegmento * (float)(Math.Sin(MathHelper.ToRadians(this.segmentoComEngrenagemEsquerdo.anguloGiro.Z))),
                                 0.0f);
            }

            // Corrige a posição do dedo esquerdo caso este ultrapasse a posição de "aberto".
            if (this.dedoEsquerdo.posReferencial.X < this.dedoEsquerdo.posDedoAberto.X || this.dedoEsquerdo.posReferencial.Y < this.dedoEsquerdo.posDedoAberto.Y)
            {
                this.dedoEsquerdo.posReferencial = this.dedoEsquerdo.posDedoAberto;
            }
        }

        /// <summary>
        /// Fecha a garra. A cada execução deste método, os dedos da garra deverão se deslocar uma distância dependente 
        /// da velocidade de rotação do motor.
        /// </summary>
        public void Fecha()
        {
            // Se dedo direito ainda não estiver em posição de "fechado"
            if (this.dedoDireito.posReferencial.X > this.dedoDireito.posDedoFechado.X && this.dedoDireito.posReferencial.Y < this.dedoDireito.posDedoFechado.Y)
            {
                // Velocidade angular do eixo do servo da garra
                this.servoDaGarra.VelocidadeAngularEixo = this.servoDaGarra.velAngularEixoMax;

                // Velocidade angular da Engrenagem do servo -> Velocidade angular da Engrenagem do eixo engrenado esquerdo
                this.segmentoComEngrenagemEsquerdo.velocidadeAng = -this.eixoEngrenagemAmortecida.numDentes * (this.eixoEngrenagemAmortecida.velocidadeAng) / this.segmentoComEngrenagemEsquerdo.numDentes;
                // Velocidade angular do segmento articulado esquerdo
                this.segmentoArticuladoEsquerdo.velocidadeAng = this.segmentoComEngrenagemEsquerdo.velocidadeAng;

                // Velocidade angular da Engrenagem do eixo engrenado esquerdo -> Velocidade angular do eixo engrenado direito
                this.segmentoComEngrenagemDireito.velocidadeAng = -this.segmentoComEngrenagemEsquerdo.velocidadeAng;
                //velocidade angular do segmento articulado direito
                this.segmentoArticuladoDireito.velocidadeAng = this.segmentoComEngrenagemDireito.velocidadeAng;


                // Ângulo do eixo do servo da garra. Lembrar que será atualizado, também, o ângulo de giro do componente físico 
                // que estiver acoplado ao eixo do servo (neste caso, a engrenagem amortecida, representada por eixoEngrenagemAmortecida)
                this.servoDaGarra.AnguloGiroEixo += this.servoDaGarra.VelocidadeAngularEixo;
                // Ângulo da Engrenagem do eixo engrenado esquerdo
                this.segmentoComEngrenagemEsquerdo.anguloGiro += this.segmentoComEngrenagemEsquerdo.velocidadeAng;
                // Ângulo do segmento articulado esquerdo
                this.segmentoArticuladoEsquerdo.anguloGiro += this.segmentoArticuladoEsquerdo.velocidadeAng;
                // Ângulo da Engrenagem do eixo engrenado direito
                this.segmentoComEngrenagemDireito.anguloGiro += this.segmentoComEngrenagemDireito.velocidadeAng;
                // Ângulo do segmento articulado direito
                this.segmentoArticuladoDireito.anguloGiro += this.segmentoArticuladoDireito.velocidadeAng;

                // Posição do dedo direito
                this.dedoDireito.posReferencial =
                    new Vector3(this.segmentoComEngrenagemDireito.tamanhoSegmento * ((float)(Math.Cos(MathHelper.ToRadians(this.segmentoComEngrenagemDireito.anguloGiro.Z))) - 1.0f),
                                this.segmentoComEngrenagemDireito.tamanhoSegmento * ((float)(Math.Sin(MathHelper.ToRadians(this.segmentoComEngrenagemDireito.anguloGiro.Z)))),
                                 0.0f);
            }
            // Corrige a posição do dedo direito caso ultrapasse a posição de "fechado"
            if (this.dedoDireito.posReferencial.X < this.dedoDireito.posDedoFechado.X || this.dedoDireito.posReferencial.Y > this.dedoDireito.posDedoFechado.Y)
            {
                this.dedoDireito.posReferencial = this.dedoDireito.posDedoFechado;
            }
            // Se dedo esquerdo ainda não estiver em posição de "fechado"
            if (this.dedoEsquerdo.posReferencial.X < this.dedoEsquerdo.posDedoFechado.X && this.dedoEsquerdo.posReferencial.Y < this.dedoEsquerdo.posDedoFechado.Y)
            {
                // Posição do dedo esquerdo
                this.dedoEsquerdo.posReferencial =
                    new Vector3( this.segmentoComEngrenagemEsquerdo.tamanhoSegmento * (1.0f - (float)(Math.Cos(MathHelper.ToRadians(this.segmentoComEngrenagemEsquerdo.anguloGiro.Z)))),
                                 -this.segmentoComEngrenagemEsquerdo.tamanhoSegmento * (float)(Math.Sin(MathHelper.ToRadians(this.segmentoComEngrenagemEsquerdo.anguloGiro.Z))),
                                 0.0f);
            }

            // Corrige a posição do dedo esquerdo caso este ultrapasse a posição de "fechado".
            if (this.dedoEsquerdo.posReferencial.X > this.dedoEsquerdo.posDedoFechado.X || this.dedoEsquerdo.posReferencial.Y > this.dedoEsquerdo.posDedoFechado.Y)
            {
                this.dedoEsquerdo.posReferencial = this.dedoEsquerdo.posDedoFechado;
            }
        }

        /// <summary>
        /// Gira a garra. Se a garra estiver segurando um objeto, o mesmo gira junto com a garra.
        /// </summary>
        public override void Girar()
        {
            base.Girar();
            
            // Se a garra estiver segurando um objeto, o mesmo terá mesma velocidade e aceleração de giro da garra,
            // bem como irá girar junto com a garra.
            if (this.objeto != null)
            {
                this.objeto.aceleracaoAng = this.aceleracaoAng;
                this.objeto.velocidadeAng = this.velocidadeAng;
                this.objeto.Girar();
            }
        }

        /// <summary>
        /// Gira a garra em sentido contrário ao método Girar(). Se a garra estiver segurando um objeto, o mesmo gira junto com a garra.
        /// </summary>
        public override void GirarInvertido()
        {
            base.GirarInvertido();
            // Se a garra estiver segurando um objeto, o mesmo terá mesma velocidade e aceleração de giro da garra,
            // bem como irá girar junto com a garra.
            if (this.objeto != null)
            {
                this.objeto.aceleracaoAng = this.aceleracaoAng;
                this.objeto.velocidadeAng = this.velocidadeAng;
                this.objeto.GirarInvertido();
            }
        }

        /// <summary>
        /// Função a ser chamada sempre que houver alguma mudança em AnguloCorrente
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void servoDaGarra_ChangedAnguloCorrente(object sender, EventArgs e)
        {
            //Estas instruções são para compensar o problema do eixo do servo de girar em torno do eixo Y da base da
            // garra, ao invés do eixo Y do servo, que é alinhado com o eixo Z da base.
            this.servoDaGarra.AnguloGiroEixo = new Vector3(this.servoDaGarra.AnguloGiroEixo.X, 
                                                           this.servoDaGarra.AnguloGiroEixo.Y,
                                                           this.servoDaGarra.anguloGiroEixoInicial.Z + this.servoDaGarra.sinalPropagacaoAnguloCorrente * this.servoDaGarra.AnguloCorrente);
            Console.WriteLine(this.servoDaGarra.sigla + ":" + this.servoDaGarra.AnguloCorrente.ToString());
            //Console.WriteLine(this.servoDaGarra.AnguloGiroEixo.ToString());

            AtualizarEngrenagensEDedos();
        }

        /// <summary>
        /// Atualiza as posições de todas as peças da garra (engrenagens e dedos)
        /// </summary>
        public void AtualizarEngrenagensEDedos()
        {

            this.eixoEngrenagemAmortecida.anguloGiro = this.servoDaGarra.eixo.anguloGiro;
            // Posição angular da Engrenagem do servo -> Posição angular da Engrenagem do eixo engrenado esquerdo
            this.segmentoComEngrenagemEsquerdo.anguloGiro = -this.eixoEngrenagemAmortecida.numDentes * (this.eixoEngrenagemAmortecida.anguloGiro) / this.segmentoComEngrenagemEsquerdo.numDentes;
            // Posição angular do segmento articulado esquerdo
            this.segmentoArticuladoEsquerdo.anguloGiro = this.segmentoComEngrenagemEsquerdo.anguloGiro;

            // Posição angular da Engrenagem do eixo engrenado esquerdo -> Posição angular do eixo engrenado direito
            this.segmentoComEngrenagemDireito.anguloGiro = -this.segmentoComEngrenagemEsquerdo.anguloGiro;
            // Posição angular do segmento articulado direito
            this.segmentoArticuladoDireito.anguloGiro = this.segmentoComEngrenagemDireito.anguloGiro;

            // Posição do dedo direito
            this.dedoDireito.posReferencial =
                new Vector3(this.segmentoComEngrenagemDireito.tamanhoSegmento * ((float)(Math.Cos(MathHelper.ToRadians(this.segmentoComEngrenagemDireito.anguloGiro.Z))) - 1.0f),
                            this.segmentoComEngrenagemDireito.tamanhoSegmento * ((float)(Math.Sin(MathHelper.ToRadians(this.segmentoComEngrenagemDireito.anguloGiro.Z)))),
                             0.0f);

            // Posição do dedo esquerdo
            this.dedoEsquerdo.posReferencial =
                new Vector3(this.segmentoComEngrenagemEsquerdo.tamanhoSegmento * (1.0f - (float)(Math.Cos(MathHelper.ToRadians(this.segmentoComEngrenagemEsquerdo.anguloGiro.Z)))),
                             -this.segmentoComEngrenagemEsquerdo.tamanhoSegmento * (float)(Math.Sin(MathHelper.ToRadians(this.segmentoComEngrenagemEsquerdo.anguloGiro.Z))),
                             0.0f);

            //// Se dedo direito ainda não estiver em posição de aberto
            //if (  (this.dedoDireito.posReferencial.X < this.dedoDireito.posDedoAberto.X && this.dedoDireito.posReferencial.Y > this.dedoDireito.posDedoAberto.Y)
            //     && (this.dedoDireito.posReferencial.X > this.dedoDireito.posDedoFechado.X && this.dedoDireito.posReferencial.Y < this.dedoDireito.posDedoFechado.Y)
            //   )
            //{
            //    this.eixoEngrenagemAmortecida.anguloGiro = this.servoDaGarra.eixo.anguloGiro;
            //    // Posição angular da Engrenagem do servo -> Posição angular da Engrenagem do eixo engrenado esquerdo
            //    this.segmentoComEngrenagemEsquerdo.anguloGiro = -this.eixoEngrenagemAmortecida.numDentes * (this.eixoEngrenagemAmortecida.anguloGiro) / this.segmentoComEngrenagemEsquerdo.numDentes;
            //    // Posição angular do segmento articulado esquerdo
            //    this.segmentoArticuladoEsquerdo.anguloGiro = this.segmentoComEngrenagemEsquerdo.anguloGiro;

            //    // Posição angular da Engrenagem do eixo engrenado esquerdo -> Posição angular do eixo engrenado direito
            //    this.segmentoComEngrenagemDireito.anguloGiro = -this.segmentoComEngrenagemEsquerdo.anguloGiro;
            //    // Posição angular do segmento articulado direito
            //    this.segmentoArticuladoDireito.anguloGiro = this.segmentoComEngrenagemDireito.anguloGiro;

            //    // Posição do dedo direito
            //    this.dedoDireito.posReferencial =
            //        new Vector3(this.segmentoComEngrenagemDireito.tamanhoSegmento * ((float)(Math.Cos(MathHelper.ToRadians(this.segmentoComEngrenagemDireito.anguloGiro.Z))) - 1.0f),
            //                    this.segmentoComEngrenagemDireito.tamanhoSegmento * ((float)(Math.Sin(MathHelper.ToRadians(this.segmentoComEngrenagemDireito.anguloGiro.Z)))),
            //                     0.0f);
            //}
            // Corrige a posição do dedo direito caso ultrapasse a posição de "aberto"
            if (this.dedoDireito.posReferencial.X > this.dedoDireito.posDedoAberto.X || this.dedoDireito.posReferencial.Y < this.dedoDireito.posDedoAberto.Y)
            {
                this.dedoDireito.posReferencial = this.dedoDireito.posDedoAberto;
                this.segmentoComEngrenagemDireito.anguloGiro.Z = (MathHelper.ToDegrees((float)Math.Acos(this.dedoDireito.posReferencial.X / this.segmentoComEngrenagemDireito.tamanhoSegmento + 1.0f)));
                this.segmentoArticuladoDireito.anguloGiro.Z = this.segmentoComEngrenagemDireito.anguloGiro.Z;
            }
            // Corrige a posição do dedo direito caso ultrapasse a posição de "fechado"
            else if (this.dedoDireito.posReferencial.X < this.dedoDireito.posDedoFechado.X || this.dedoDireito.posReferencial.Y > this.dedoDireito.posDedoFechado.Y)
            {
                this.dedoDireito.posReferencial = this.dedoDireito.posDedoFechado;
                this.segmentoComEngrenagemDireito.anguloGiro.Z = (MathHelper.ToDegrees((float)Math.Acos(this.dedoDireito.posReferencial.X / this.segmentoComEngrenagemDireito.tamanhoSegmento + 1.0f)));
                this.segmentoArticuladoDireito.anguloGiro.Z = this.segmentoComEngrenagemDireito.anguloGiro.Z;
            }

            //// Se dedo esquerdo ainda não estiver em posição de "aberto"
            //if (this.dedoEsquerdo.posReferencial.X > this.dedoEsquerdo.posDedoAberto.X && this.dedoEsquerdo.posReferencial.Y > this.dedoEsquerdo.posDedoAberto.Y)
            //{
            //    // Posição do dedo esquerdo
            //    this.dedoEsquerdo.posReferencial =
            //        new Vector3(this.segmentoComEngrenagemEsquerdo.tamanhoSegmento * (1.0f - (float)(Math.Cos(MathHelper.ToRadians(this.segmentoComEngrenagemEsquerdo.anguloGiro.Z)))),
            //                     -this.segmentoComEngrenagemEsquerdo.tamanhoSegmento * (float)(Math.Sin(MathHelper.ToRadians(this.segmentoComEngrenagemEsquerdo.anguloGiro.Z))),
            //                     0.0f);
            //}

            // Corrige a posição do dedo esquerdo caso este ultrapasse a posição de "aberto".
            if (this.dedoEsquerdo.posReferencial.X < this.dedoEsquerdo.posDedoAberto.X || this.dedoEsquerdo.posReferencial.Y < this.dedoEsquerdo.posDedoAberto.Y)
            {
                this.dedoEsquerdo.posReferencial = this.dedoEsquerdo.posDedoAberto;
                this.segmentoComEngrenagemEsquerdo.anguloGiro.Z = -(MathHelper.ToDegrees((float)Math.Acos(1.0f - this.dedoEsquerdo.posReferencial.X / this.segmentoComEngrenagemEsquerdo.tamanhoSegmento)));
                //this.segmentoComEngrenagemEsquerdo.anguloGiro.Z = (MathHelper.ToDegrees((float)Math.Asin(this.dedoEsquerdo.posReferencial.Y / this.segmentoComEngrenagemEsquerdo.tamanhoSegmento)));
                this.eixoEngrenagemAmortecida.anguloGiro.Z = -this.segmentoComEngrenagemEsquerdo.numDentes * this.segmentoComEngrenagemEsquerdo.anguloGiro.Z / this.eixoEngrenagemAmortecida.numDentes;
                this.segmentoArticuladoEsquerdo.anguloGiro.Z = this.segmentoComEngrenagemEsquerdo.anguloGiro.Z;
            }
            // Corrige a posição do dedo esquerdo caso este ultrapasse a posição de "fechado".
            else if (this.dedoEsquerdo.posReferencial.X > this.dedoEsquerdo.posDedoFechado.X || this.dedoEsquerdo.posReferencial.Y > this.dedoEsquerdo.posDedoFechado.Y)
            {
                this.dedoEsquerdo.posReferencial = this.dedoEsquerdo.posDedoFechado;
                this.segmentoComEngrenagemEsquerdo.anguloGiro.Z = -(MathHelper.ToDegrees((float)Math.Acos(1.0f - this.dedoEsquerdo.posReferencial.X / this.segmentoComEngrenagemEsquerdo.tamanhoSegmento)));
                //this.segmentoComEngrenagemEsquerdo.anguloGiro.Z = (MathHelper.ToDegrees((float)Math.Asin(this.dedoEsquerdo.posReferencial.Y / this.segmentoComEngrenagemEsquerdo.tamanhoSegmento)));
                this.eixoEngrenagemAmortecida.anguloGiro.Z = -this.segmentoComEngrenagemEsquerdo.numDentes * this.segmentoComEngrenagemEsquerdo.anguloGiro.Z / this.eixoEngrenagemAmortecida.numDentes;
                this.segmentoArticuladoEsquerdo.anguloGiro.Z = this.segmentoComEngrenagemEsquerdo.anguloGiro.Z;
                
            }
            Console.WriteLine("Ângulo eixo engr. esq.:" + this.segmentoComEngrenagemEsquerdo.anguloGiro);

        }

        /// <summary>
        /// Função para gerar as matrizesGarra dos componentes envolvidos na garra.
        /// </summary>
        /// <param name="mt">Matriz do componente físico ao qual a garra está acoplada</param>
        /// <returns>Um vetor de matrizesGarra para os componentes físicos da garra.</returns>
        public List<Matrix> Matrizes(Matrix mt)
        {
            List<Matrix> m = new List<Matrix>();
            //Matrix mServo;

            // Base da garra
            m.Add(this.MatrizRotacaoX() * this.MatrizRotacaoY() * this.MatrizRotacaoZ() * this.MatrizTranslacao() * mt);

            //// Servo
            //mServo =  this.servoDaGarra.MatrizRotacaoX() * this.servoDaGarra.MatrizRotacaoY() * this.servoDaGarra.MatrizRotacaoZ() * this.servoDaGarra.MatrizTranslacao() * m[0];
            //m.AddRange(this.servoDaGarra.Matrizes(mServo));

            // Engrenagem amortecida do eixo do servo
            m.Add(this.eixoEngrenagemAmortecida.MatrizRotacaoZ_ve() * m[0]);

            // Segmento engrenado esquerdo
            m.Add(this.segmentoComEngrenagemEsquerdo.MatrizRotacaoZ_ve() * m[0]);

            // Segmento engrenado direito
            m.Add(this.segmentoComEngrenagemDireito.MatrizRotacaoZ_ve() * m[0]);

            // Segmento esquerdo
            m.Add(this.segmentoArticuladoEsquerdo.MatrizRotacaoZ_ve() * m[0]);

            // Segmento direito
            m.Add(this.segmentoArticuladoDireito.MatrizRotacaoZ_ve() * m[0]);

            // Dedo esquerdo
            m.Add(this.dedoEsquerdo.MatrizTranslacao() * m[0]);

            // Dedo direito
            m.Add(this.dedoDireito.MatrizTranslacao() * m[0]);

            return m;
        }

    }
}
