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
using System.Timers;

namespace BracoRobo.Classes
{
    /// <summary>
    /// Classe para representar a placa Mini Maestro 24 da Pololu
    /// </summary>
    public class PlacaMiniMaestro24
    {
        /// <summary>
        /// Atributo que serve para contagem de tempo e para ativar o evento de alteração dos canais
        /// da placa mini maestro.
        /// </summary>
        Timer timer;

        /// <summary>
        /// Atributo que conta 80 milissegundos, ou seja, conta 8 vezes os estouros do timer.
        /// </summary>
        private float conta80ms = 0;

        /// <summary>
        /// Vetor que contém as informações referentes aos canais da placa mini maestro
        /// </summary>
        public CanalMiniMaestro[] canais;

        /// <summary>
        /// Variável que contém o estado de movimento dos canais. Se pelo menos 1 estiver mudando o position,
        /// movingState vale 0x01; caso contrário, 0x00.
        /// </summary>
        public byte movingState = 0x00;

        /// <summary>
        /// Construtor padrão
        /// </summary>
        public PlacaMiniMaestro24()
        {
            this.canais = new CanalMiniMaestro[6];

            for (int i = 0; i < 6; i++)
            {
                this.canais[i] = new CanalMiniMaestro();
            }

            this.timer = new Timer(10);
            this.timer.Elapsed +=new ElapsedEventHandler(timer_Elapsed);
            this.timer.AutoReset = false; // true para o timer estourar sempre que contar o tempo, false para o timer
                                         // desabilitar após o primeiro estouro.
        }

        /// <summary>
        /// Rotina de evento do timer que será acionada sempre que o timer estourar
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void  timer_Elapsed(object sender, ElapsedEventArgs e)
        {
            byte i;
            bool todosParados = true;

            conta80ms++;

            float target, position, positionMedia;
            float sinalVel;
            bool inverteSinalVelocidade;

            bool contou80ms = false;

            if (conta80ms >= 8)
            {
                contou80ms = true;
                conta80ms = 0;
            }
            
            for (i = 0; i < 6; i++)
            {
                target = canais[i].Target;
                position = canais[i].Position;
                positionMedia = canais[i].positionMedia;
                sinalVel = canais[i].sinalVel;
                inverteSinalVelocidade = canais[i].inverteSinalVelocidade;

                if (((sinalVel > 0 || (inverteSinalVelocidade && sinalVel < 0)) && position < target) ||
                    ((sinalVel < 0 || (inverteSinalVelocidade && sinalVel > 0)) && position > target) 
                  )
                {
                    // Tratamento da aceleração
                    if (canais[i].acceleration > 0)
                    {
                        if (!inverteSinalVelocidade &&
                             ((sinalVel > 0 && position >= positionMedia) ||
                              (sinalVel < 0 && position <= positionMedia)
                             )
                           )
                        {
                            canais[i].accelerationCurr = -1 * canais[i].sinalAcel * canais[i].acceleration;
                        }

                        if (contou80ms) // se já passou 80ms (para a aceleração)
                        {
                            canais[i].speedCurrNoLimit += canais[i].accelerationCurr;
                            
                            if (inverteSinalVelocidade &&
                                ((canais[i].sinalAcel > 0 && canais[i].speedCurrNoLimit >= 0) ||
                                 (canais[i].sinalAcel < 0 && canais[i].speedCurrNoLimit <= 0)
                                )
                               )
                            {
                                // Aqui ocorre a inversão do sinal de velocidade do canal
                                canais[i].speedCurrNoLimit = 0;
                                canais[i].inverteSinalVelocidade = false;
                                sinalVel = -1 * sinalVel;
                                canais[i].sinalVel = sinalVel;
                            }

                            if (Math.Abs(canais[i].speedCurrNoLimit) <= canais[i].speed || canais[i].speed == 0)
                            {
                                canais[i].speedCurr = canais[i].speedCurrNoLimit;
                            }
                            else
                            {
                                canais[i].speedCurr = canais[i].sinalVel * canais[i].speed;
                            }
                        }
                    }

                    if (Math.Abs(canais[i].speedCurr) > 0 && Math.Abs(target - position) >= Math.Abs(canais[i].speedCurr))
                    {
                        position = position + canais[i].speedCurr;

                        if (position < canais[i].min)
                        {
                            position = canais[i].min;

                            if (inverteSinalVelocidade)
                            {
                                // Aqui ocorre a inversão do sinal de velocidade do canal
                                canais[i].speedCurrNoLimit = 0;
                                canais[i].speedCurr = 0;
                                canais[i].inverteSinalVelocidade = false;
                                sinalVel = -1 * sinalVel;
                                canais[i].sinalVel = sinalVel;
                            }
                        }
                        else if (position > canais[i].max)
                        {
                            position = canais[i].max;

                            if (inverteSinalVelocidade)
                            {
                                // Aqui ocorre a inversão do sinal de velocidade do canal
                                canais[i].speedCurrNoLimit = 0;
                                canais[i].speedCurr = 0;
                                canais[i].inverteSinalVelocidade = false;
                                sinalVel = -1 * sinalVel;
                                canais[i].sinalVel = sinalVel;
                            }
                        }

                        canais[i].Position = position;
                    }
                    else if (Math.Abs(canais[i].speedCurr) > 0)
                    {
                        canais[i].Position = target;
                        canais[i].speedCurr = 0;
                        canais[i].speedCurrNoLimit = 0;
                        canais[i].sinalVel = 0;
                        canais[i].sinalAcel = 0;
                        canais[i].inverteSinalVelocidade = false;
                    }
                    else if (!inverteSinalVelocidade &&
                              ((sinalVel > 0 && position > positionMedia) ||
                               (sinalVel < 0 && position < positionMedia)
                              )
                            )
                    {
                        canais[i].Position = target;
                        canais[i].speedCurr = 0;
                        canais[i].speedCurrNoLimit = 0;
                        canais[i].sinalVel = 0;
                        canais[i].sinalAcel = 0;
                    }

                    todosParados = false;
                }
                else if ((sinalVel > 0 && position > target) ||
                         (sinalVel < 0 && position < target) 
                        )
                        
                {                    
                    canais[i].Position = target;
                    canais[i].speedCurr = 0;
                    canais[i].speedCurrNoLimit = 0;
                    canais[i].sinalVel = 0;
                    canais[i].sinalAcel = 0;
                    canais[i].inverteSinalVelocidade = false;
                    
                    todosParados = false;
                }
                else
                {
                    todosParados = todosParados && true;
                }
            }

            
            if (todosParados)
            {
                //timer.Stop();
                Console.WriteLine("Timer Stop");
                movingState = 0x00;
            }
            else
            {
                timer.Start();
                movingState = 0x01;
            }
        }

        /// <summary>
        /// Seta o Target do canal, ativando o timer
        /// </summary>
        /// <param name="canal">canal do Target a ser setado</param>
        /// <param name="target4">valor do Target a ser setado</param>
        public void SetTarget(byte canal, float target4)
        {
            if (SetarTarget(canal, target4))
            {
                movingState = 0x01;
                timer.Start();
            }
        }

        /// <summary>
        /// Seta o Target do canal, sem ativar o timer.
        /// </summary>
        /// <param name="canal">canal do Target a ser setado</param>
        /// <param name="target4">valor do Target a ser setado</param>
        /// <returns>true se o target setado for diferente da posição atual, ou false, caso contrário</returns>
        public bool SetarTarget(byte canal, float target4)
        {
            float speed, acceleration, position, target;

            canais[canal].Target = target4 * 4;

            speed = canais[canal].speed;
            acceleration = canais[canal].acceleration;
            position = canais[canal].Position;
            target = canais[canal].Target;

            conta80ms = 0;

            if (position == 0)
            {
                canais[canal].Position = target;
                position = target;
            }

            if (target == 0)
            {
                canais[canal].Position = 0;
                position = 0;
            }
            else if (target > canais[canal].max)
            {
                target = canais[canal].max;
                canais[canal].Target = target;
            }
            else if (target < canais[canal].min)
            {
                target = canais[canal].min;
                canais[canal].Target = target;
            }


            if (target > position) // movimento positivo
            {
                if (acceleration > 0) // com aceleração
                {
                    canais[canal].sinalAcel = 1;
                    canais[canal].accelerationCurr = canais[canal].sinalAcel * acceleration;

                    if (canais[canal].speedCurrNoLimit >= 0.0f)
                    {
                        canais[canal].sinalVel = 1;
                        canais[canal].inverteSinalVelocidade = false;
                    }
                    else
                    {
                        canais[canal].speedCurrNoLimit = canais[canal].speedCurr;
                        canais[canal].inverteSinalVelocidade = true;
                    }

                    canais[canal].positionMedia = (target + position) / 2;                    
                }
                else // sem aceleração
                {
                    canais[canal].sinalAcel = 0;
                    canais[canal].accelerationCurr = 0;

                    if (canais[canal].speed > 0) // com velocidade limite
                    {
                        canais[canal].sinalVel = 1;
                        canais[canal].speedCurr = canais[canal].sinalVel * speed;
                    }
                    else // sem velocidade limite
                    {
                        canais[canal].sinalVel = 0;
                        canais[canal].speedCurr = 0;
                        canais[canal].Position = target; // Vai direto para a posição alvo

                        return false;
                    }
                    canais[canal].inverteSinalVelocidade = false;
                    canais[canal].speedCurrNoLimit = canais[canal].speedCurr;
                    canais[canal].positionMedia = (target + position) / 2;
                }

                return true;
            }
            else if (target < position) // movimento negativo
            {
                if (acceleration > 0) // com aceleração
                {
                    canais[canal].sinalAcel = -1;
                    canais[canal].accelerationCurr = canais[canal].sinalAcel * acceleration;

                    if (canais[canal].speedCurrNoLimit > 0.0f)
                    {
                        canais[canal].speedCurrNoLimit = canais[canal].speedCurr;
                        canais[canal].inverteSinalVelocidade = true;
                    }
                    else
                    {
                        canais[canal].sinalVel = -1;
                        canais[canal].inverteSinalVelocidade = false;
                    }

                    canais[canal].positionMedia = (target + position) / 2;                    
                }
                else // sem aceleração
                {
                    canais[canal].sinalAcel = 0;
                    canais[canal].accelerationCurr = 0;

                    if (canais[canal].speed > 0) // com velocidade limite
                    {
                        canais[canal].sinalVel = -1;
                        canais[canal].speedCurr = canais[canal].sinalVel * speed;
                    }
                    else // sem velocidade limite
                    {
                        canais[canal].sinalVel = 0;
                        canais[canal].speedCurr = 0;
                        canais[canal].Position = target; // Vai direto para a posição alvo

                        return false;
                    }
                    canais[canal].inverteSinalVelocidade = false;
                    canais[canal].speedCurrNoLimit = canais[canal].speedCurr;
                    canais[canal].positionMedia = (target + position) / 2;
                }

                return true;
            }
            else // sem movimento
            {
                canais[canal].sinalVel = 0;
                canais[canal].sinalAcel = 0;
                canais[canal].accelerationCurr = 0;
                canais[canal].speedCurr = 0;
                canais[canal].inverteSinalVelocidade = false;
                canais[canal].speedCurrNoLimit = 0;
                canais[canal].positionMedia = (target + position) / 2;

                return false;
            }
        }

        /// <summary>
        /// Seta os targets de vários canais, do primeiro canal especificado em primeiroCanal
        /// até o último canal (primeiroCanal+numTargets)
        /// </summary>
        /// <param name="numTargets">Quantidade de targets (ou canais) a serem setados</param>
        /// <param name="primeiroCanal">primeiro canal da sequência a ser setado</param>
        /// <param name="targets">vetor contendo os targets a serem setados</param>
        public void SetMultipleTargets(byte numTargets, byte primeiroCanal, float[] targets)
        {
            byte i;
            bool result = false;
            
            for (i = primeiroCanal; i < primeiroCanal + numTargets; i++)
            {
                result = SetarTarget(i, targets[i]) || result; // SetarTarget deve vir antes, pois se result for true, SetarTarget não será executada.
            }

            if (result)
            {
                movingState = 0x01;
                timer.Start();
            }
        }

        /// <summary>
        /// Seta a velocidade máxima (de variação do Target) do canal, em unidades de (0.25 μs)/(10 ms).
        /// </summary>
        /// <param name="canal">Canal a ter sua velocidade máxima alterada</param>
        /// <param name="vel">Valor da velocidade a ser setada, em unidades de (0.25 μs)/(10 ms)</param>
        /// <returns>true se a velocidade foi corretamente setada, false caso contrário.</returns>
        public bool SetSpeed(byte canal, UInt16 vel)
        {
            canais[canal].speed = vel;
            return true;
        }
        /// <summary>
        /// Seta a aceleração do canal, em unidades de (0.25 μs)/(10 ms)/(80 ms)
        /// </summary>
        /// <param name="canal">Canal a ter sua aceleração alterada</param>
        /// <param name="accel">Valor da aceleração a ser setada, em unidades de (0.25 μs)/(10 ms)/(80 ms)</param>
        /// <returns>true se a velocidade foi corretamente setada, false caso contrário.</returns>
        public bool SetAcceleration(byte canal, UInt16 accel)
        {
            canais[canal].acceleration = accel;
            return true;
        }

        /// <summary>
        /// Obtém da placa Mini Maestro a posição corrente do canal, em unidades de 0.25us.
        /// </summary>
        /// <param name="canal">Canal do qual se quer saber a posição</param>
        /// <returns>A posição em unidades de 0.25us</returns>
        public float GetPosition(byte canal)
        {
            return canais[canal].Position;
        }

        /// <summary>
        /// Obtém da placa Mini Maestro o estado de movimento dos canais. Enquanto houver pelo menos 1 canal tendo
        /// sua posição alterada, esta função irá avisar que há movimento.
        /// </summary>
        /// <returns>0x01 (um), se pelo menos 1 canal estiver alterando posição, ou 0x00 (zero), caso contrário</returns>
        public byte GetMovingState()
        {
            return movingState;
        }

        /// <summary>
        /// Obtém os flags de erro da placa Mini Maestro.
        /// </summary>
        /// <returns>Retorna 0x0000. Futuramente será feita simulação destes erros</returns>
        public UInt16 GetErrors()
        {
            UInt16 error = 0x0000;
            return error;
        }

    }
}
