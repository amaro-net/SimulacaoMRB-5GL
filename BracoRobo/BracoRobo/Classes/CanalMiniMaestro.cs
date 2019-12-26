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
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BracoRobo.Classes
{
    /// <summary>
    /// Classe que representa um canal das placas Mini Maestro da Pololu
    /// </summary>
    public class CanalMiniMaestro
    {
        /// <summary>
        /// Posição corrente do canal em unidades de 0.25 de microssegundo
        /// </summary>
        private float position;

        /// <summary>
        /// Posição corrente do canal em unidades de 0.25 de microssegundo (property)
        /// </summary>
        public float Position
        {
            get { return position; }
            set
            {
                position = value;
                if (this.servo != null)
                {
                    // Converte position para microssegundos
                    this.servo.TempoPulsoCorrente = this.position / 4;
                }
            }
        }
        /// <summary>
        /// Posição em que o canal pára de acelerar
        /// </summary>
        public float positionFinalAcel;
        /// <summary>
        /// Posição em que o canal começa a desacelerar
        /// </summary>
        public float positionInicialDesacel;
        /// <summary>
        /// Posição em que o canal muda da aceleração para a desaceleração. Usado quando os instante
        /// de fim de aceleração (tempoAcelerando) é maior ou igual ao instante de inicio de 
        /// desaceleração (tempoIniDesacel).
        /// </summary>
        public float positionMedia;
        /// <summary>
        /// Posição alvo do canal em unidades de 0.25 de microssegundo
        /// </summary>
        private float target;

        /// <summary>
        /// Posição alvo do canal em unidades de 0.25 de microssegundo (property).
        /// </summary>
        public float Target
        {
            get { return target; }
            set 
            {
                target = value;
                if (this.servo != null)
                {
                    this.servo.tempoPulsoAlvo = this.target / 4; // converte para us
                }
            }
        }
        /// <summary>
        /// Menor posição que o canal pode assumir, em unidades de 0.25 de microssegundo
        /// </summary>
        public float min;
        /// <summary>
        /// Maior posição que o canal pode assumir, em unidades de 0.25 de microssegundo
        /// </summary>
        public float max;
        /// <summary>
        /// Velocidade máxima de variação da posição do canal em unidades de (0.25 μs)/(10 ms).
        /// </summary>
        public float speed;
        /// <summary>
        /// Aceleraçao da posição do canal em unidades de (0.25 μs)/(10 ms)/(80 ms).
        /// </summary>
        public float acceleration;
        /// <summary>
        /// Variável para determinar se a velocidade será positiva ou negativa ao alterar a posição do canal
        /// </summary>
        public float sinalVel = 0;
        /// <summary>
        /// Variável para determinar se a aceleração será positiva ou negativa ao alterar a velocidade do canal
        /// </summary>
        public float sinalAcel = 0;
        /// <summary>
        /// Velocidade corrente do canal. Pode variar com o tempo. Tem como valor máximo o valor da velocidade armazenado em speed.
        /// </summary>
        public float speedCurr = 0;
        /// <summary>
        /// Aceleração corrente do canal. Pode ser nula, negativa ou positiva, dependendo do tempo. Será
        /// representada em unidades de (0.25 μs)/(10 ms)/(80 ms).
        /// </summary>
        public float accelerationCurr = 0;
        /// <summary>
        /// Servo que o canal controla
        /// </summary>
        public Servomotor servo;

        /// <summary>
        /// Velocidade corrente do canal, sem o limite de speed. Usada no controle de aceleração.
        /// </summary>
        public float speedCurrNoLimit;
        /// <summary>
        /// Flag para indicar que deverá haver uma inversão de sinal da velocidade do canal. Usado quando
        /// o canal possui aceleração maior que zero. Este flag vai para true quando, em movimento, a posição
        /// alvo é alterada de forma que o movimento é invertido, ou seja, quando o movimento for positivo e
        /// passar a ser negativo, ou quando o movimento for negativo e passar a ser positivo.
        /// </summary>
        public bool inverteSinalVelocidade = false;

        /// <summary>
        /// Construtor padrão do canal da Mini Maestro
        /// </summary>
        public CanalMiniMaestro()
        {

        }

    }
}
