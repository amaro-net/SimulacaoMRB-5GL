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
using Microsoft.Xna.Framework;

namespace BracoRobo.Classes
{
    /// <summary>
    /// Classe para guardar dados de uma câmera
    /// </summary>
    public class Camera
    {
        /// <summary>
        /// Distância absoluta da câmera para o ponto focal (entre posicaoCamera e posicaoApontada)
        /// </summary>
        public float distanciaCamera = 98.0f;
        /// <summary>
        /// Distância absoluta inicial da câmera para o ponto focal (entre posicaoCamera e posicaoApontada)
        /// </summary>
        public float distanciaCameraInicial = 98.0f;
        /// <summary>
        /// Posição da câmera. As coordenadas são em relação ao referencial definido pela matriz world.
        /// </summary>
        public Vector3 posicaoCamera = new Vector3(0.0f, 20.0f, -20.0f);
        /// <summary>
        /// Posição para onde a câmera está olhando
        /// </summary>
        public Vector3 posicaoApontada = new Vector3(0.0f, 26.0f, 0.0f);
        /// <summary>
        /// Posição para onde a câmera estava olhando
        /// </summary>
        public Vector3 posicaoApontadaInicial = new Vector3(0.0f, 26.0f, 0.0f);
        /// <summary>
        /// Ângulo em graus do giro horizontal da câmera
        /// </summary>
        public float azimuteCamera = 30.0f;
        /// <summary>
        /// Ângulo inicial em graus do giro horizontal da câmera
        /// </summary>
        public float azimuteCameraInicial = 30.0f;
        /// <summary>
        /// Ângulo em graus do giro vertical da câmera
        /// </summary>
        public float elevacaoCamera = 45.0f;
        /// <summary>
        /// Ângulo inicial em graus do giro vertical da câmera
        /// </summary>
        public float elevacaoCameraInicial = 45.0f;

        /// <summary>
        /// Atualiza a posição da câmera
        /// </summary>
        public void UpdatePosicao()
        {
            posicaoCamera = new Vector3(distanciaCamera * (float)(Math.Sin(MathHelper.ToRadians(elevacaoCamera)) * Math.Cos(MathHelper.ToRadians(azimuteCamera))),
                                        distanciaCamera * (float)Math.Cos(MathHelper.ToRadians(elevacaoCamera)),
                                        distanciaCamera * (float)(Math.Sin(MathHelper.ToRadians(elevacaoCamera)) * Math.Sin(MathHelper.ToRadians(azimuteCamera)))
                                        );
            posicaoCamera += posicaoApontada;
        }

        /// <summary>
        /// Restaura a câmera para sua posição inicial
        /// </summary>
        public void ResetaPosicao()
        {
            this.distanciaCamera = this.distanciaCameraInicial;
            this.posicaoApontada = this.posicaoApontadaInicial;
            this.azimuteCamera = this.azimuteCameraInicial;
            this.elevacaoCamera = this.elevacaoCameraInicial;
        }

        /// <summary>
        /// Matriz de posicionamento da câmera
        /// </summary>
        /// <returns></returns>
        public Matrix MatrizView()
        {
            return Matrix.CreateLookAt(posicaoCamera, posicaoApontada, new Vector3(0.0f, 1.0f, 0.0f));
        }
    }
}
