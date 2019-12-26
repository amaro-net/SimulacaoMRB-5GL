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
    /// Enumeração dos eixos cartesianos coordenados
    /// </summary>
    public enum EixoCartesiano
    {
        /// <summary>
        /// Eixo do motor no sentido negativo de X
        /// </summary>
        Xneg = -1,
        /// <summary>
        /// Eixo do motor no sentido negativo de Y
        /// </summary>
        Yneg = -2,
        /// <summary>
        /// Eixo do motor no sentido negativo de Z
        /// </summary>
        Zneg = -3,
        /// <summary>
        /// Eixo do motor sem sentido em nenhum dos eixos cartesianos
        /// </summary>
        nenhum = 0, 
        /// <summary>
        /// Eixo do motor no sentido positivo de X
        /// </summary>
        Xpos = 1, 
        /// <summary>
        /// Eixo do motor no sentido positivo de Y
        /// </summary>
        Ypos = 2, 
        /// <summary>
        /// Eixo do motor no sentido positivo de Z
        /// </summary>
        Zpos = 3
    }

    /// <summary>
    /// Representa um motor rotacional genérico.
    /// </summary>
    public class Motor : ComponenteFisico
    {

        /**** Componentes físicos do motor ****/
        /// <summary>
        /// Componente físico correspondente ao corpo do motor. É considerado como sendo a base fixa do motor em relação ao referencial local
        /// </summary>
        public Base corpo;
        /// <summary>
        /// Componente físico correspondente ao eixo giratório do motor (parte móvel do motor)
        /// </summary>
        public Base eixo;

        /* Atributos do motor */ 
        /// <summary>
        /// Posição do eixo do motor
        /// </summary>
        public Vector3 centroEixoGiro;
        /// <summary>
        /// Ângulo que o eixo do motor girou
        /// </summary>
        private Vector3 anguloGiroEixo;
        
        /// <summary>
        /// Ângulo em graus que o eixo do motor girou, em X, Y ou Z. Note que, junto com o eixo, irá girar, também, o objeto (cargaDeAtuacao) que estiver acoplado a ele.
        /// Recomenda-se que o eixo do esteja paralelo a apenas 1 eixo (X, Y ou Z).
        /// </summary>
        public Vector3 AnguloGiroEixo
        {
            get { return this.anguloGiroEixo; }
            set { 
                this.anguloGiroEixo = value;
                this.eixo.anguloGiro = this.anguloGiroEixo;
            }
        }
        /// <summary>
        /// Ângulo em graus (em X, Y e/ou Z) que o eixo assume ao iniciar a simulação.
        /// </summary>
        public Vector3 anguloGiroEixoInicial;
        /// <summary>
        /// Velocidade de giro do eixo do motor
        /// </summary>
        private Vector3 velocidadeAngularEixo;
        /// <summary>
        /// Velocidade de giro do eixo do motor, em X, Y ou Z. Note que a velocidade de giro do eixo será transmitida para o objeto (cargaDeAtuacao) que estiver acoplado ao motor.
        /// Recomenda-se que o eixo do motor esteja paralelo a apenas 1 eixo (X, Y ou Z).
        /// </summary>
        public Vector3 VelocidadeAngularEixo
        {
            get { return velocidadeAngularEixo; }
            set {
                this.velocidadeAngularEixo = value;
                this.eixo.velocidadeAng = this.velocidadeAngularEixo;
            }
        }
        /// <summary>
        /// Velocidade máxima (em módulo) do giro do eixo em torno de X, Y ou Z
        /// </summary>
        public Vector3 velAngularEixoMax;
        /// <summary>
        /// Aceleração de giro do eixo do motor
        /// </summary>
        private Vector3 aceleracaoGiroEixo;
        /// <summary>
        /// Aceleração de giro do eixo do motor, em X, Y ou Z. Note que a aceleração de giro do eixo será transmitida para o objeto (cargaDeAtuacao) que estiver acoplado ao motor.
        /// Recomenda-se que o eixo do motor esteja paralelo a apenas 1 eixo (X, Y ou Z).
        /// </summary>
        public Vector3 AceleracaoGiroEixo
        {
            get { return aceleracaoGiroEixo; }
            set {
                this.aceleracaoGiroEixo = value;
                this.eixo.aceleracaoAng = this.aceleracaoGiroEixo;
            }
        }
        /// <summary>
        /// Objeto sobre o qual o motor irá atuar.
        /// </summary>
        public ComponenteFisico cargaDeAtuacao;
        /// <summary>
        /// Objeto no qual o motor está fixado.
        /// </summary>
        public ComponenteFisico baseDeFixacao;
        /// <summary>
        /// Eixo cartesiano da carga de atuação ao qual o eixo do motor está alinhado
        /// </summary>
        public EixoCartesiano eixoCartesianoEixoGiro = EixoCartesiano.Ypos;

        /// <summary>
        /// Construtor padrão.
        /// </summary>
        public Motor()
        {
            this.corpo = new Base();
            this.eixo = new Base();
        }
        /// <summary>
        /// Método para especificar o componente físico onde o motor será fixado.
        /// </summary>
        /// <param name="baseFix">Objeto do tipo ComponenteFisico ou de qualquer tipo herdado deste</param>
        public void FixarEm(ComponenteFisico baseFix)
        {
            this.baseDeFixacao = baseFix;
        }
        /// <summary>
        /// Método para especificar o componente físico que será acoplado ao eixo do motor.
        /// </summary>
        /// <param name="carga">Objeto do tipo ComponenteFisico ou de qualquer tipo herdado deste</param>
        public void AcoplarAoEixo(ComponenteFisico carga)
        {
            this.cargaDeAtuacao = carga;
        }

        /// <summary>
        /// Método para especificar o componente físico que será acoplado ao eixo do motor. Nesta
        /// sobrecarga de método, deve ser especificado a qual eixo do componente físico (X, Y ou Z) o 
        /// eixo do motor deve estar alinhado. Lembrar que o eixo do motor é alinhado sempre ao eixo Y
        /// positivo local. Se o objeto for plotado sem fazer as rotações após o eixo, o objeto fica
        /// alinhado ao eixo Y do eixo do motor.
        /// </summary>
        /// <param name="carga">Objeto do tipo ComponenteFisico ou de qualquer tipo herdado deste</param>
        /// <param name="eixo">Eixo cartesiano da carga ao qual o eixo do motor deve estar alinhado</param>
        public void AcoplarAoEixo(ComponenteFisico carga, EixoCartesiano eixo)
        {
            this.cargaDeAtuacao = carga;
            this.eixoCartesianoEixoGiro = eixo;
            if (this.cargaDeAtuacao != null)
            {
                switch (this.eixoCartesianoEixoGiro)
                {
                    case EixoCartesiano.Xpos:
                        this.cargaDeAtuacao.anguloGiro = new Vector3(0.0f, 0.0f, 90.0f);
                        break;

                    case EixoCartesiano.Ypos:
                        this.cargaDeAtuacao.anguloGiro = new Vector3(0.0f, 0.0f, 0.0f);
                        break;

                    case EixoCartesiano.Zpos:
                        this.cargaDeAtuacao.anguloGiro = new Vector3(90.0f, 0.0f, 0.0f);
                        break;

                    case EixoCartesiano.Xneg:
                        this.cargaDeAtuacao.anguloGiro = new Vector3(0.0f, 0.0f, -90.0f);
                        break;

                    case EixoCartesiano.Yneg:
                        this.cargaDeAtuacao.anguloGiro = new Vector3(-180.0f, 0.0f, 0.0f);
                        break;

                    case EixoCartesiano.Zneg:
                        this.cargaDeAtuacao.anguloGiro = new Vector3(-90.0f, 0.0f, 0.0f);
                        break;

                }
            }
        }

        /// <summary>
        /// Função para gerar as matrizesGarra dos componentes envolvidos no motor.
        /// </summary>
        /// <param name="mt">Matriz do componente físico ao qual o servo está acoplado</param>
        /// <returns>Um vetor de matrizesGarra para os componentes físicos da garra.</returns>
        public List<Matrix> Matrizes(Matrix mt)
        {
            List<Matrix> m = new List<Matrix>();

            m.Add(this.corpo.MatrizRotacaoX() * this.corpo.MatrizRotacaoY() * this.corpo.MatrizRotacaoZ() * this.MatrizTranslacao() * mt);
            m.Add(this.eixo.MatrizRotacaoX() * this.eixo.MatrizRotacaoY() * this.eixo.MatrizRotacaoZ() * this.MatrizTranslacao() * mt);

            return m;
        }

        /// <summary>
        /// Retorna o momento de inércia do motor (não implementado)
        /// </summary>
        /// <returns></returns>
        public override float MomentoInercia()
        {
            throw new System.NotImplementedException();
        }
    }
}
 