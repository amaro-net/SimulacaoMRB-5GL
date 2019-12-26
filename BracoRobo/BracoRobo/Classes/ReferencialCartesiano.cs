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
using Microsoft.Xna.Framework.Graphics;

namespace BracoRobo.Classes
{
    /// <summary>
    /// Classe que representa um referencial de eixos cartesianos.
    /// </summary>
    public class ReferencialCartesiano
    {
        /// <summary>
        /// Modelo 3D do referencial cartesiano
        /// </summary>
        public Model modelo;

        /// <summary>
        /// Tamanho do eixo X, sem a ponta da seta
        /// </summary>
        private float tamanhoEixoX;
        /// <summary>
        /// Tamanho do eixo Y, sem a ponta da seta (property)
        /// </summary>
        public float TamanhoEixoX
        {
            get { return tamanhoEixoX; }
            set 
            {
                tamanhoEixoX = value;
                posicaoPontaSetaEixoX.Z = posicaoPontaSetaEixoXOriginal.Z - (tamanhoEixoX - tamanhoEixoXOriginal);
            }
        }
        /// <summary>
        /// Tamanho do eixo Y, sem a ponta da seta
        /// </summary>
        private float tamanhoEixoY;
        /// <summary>
        /// Tamanho do eixo Y, sem a ponta da seta (property)
        /// </summary>
        public float TamanhoEixoY
        {
            get { return tamanhoEixoY; }
            set 
            { 
                tamanhoEixoY = value;
                posicaoPontaSetaEixoY.X = posicaoPontaSetaEixoYOriginal.X - (tamanhoEixoY - tamanhoEixoYOriginal);
            }
        }
        /// <summary>
        /// Tamanho do eixo Z, sem a ponta da seta
        /// </summary>
        private float tamanhoEixoZ;
        /// <summary>
        /// Tamanho do eixo Z, sem a ponta da seta (property)
        /// </summary>
        public float TamanhoEixoZ
        {
            get { return tamanhoEixoZ; }
            set 
            {
                tamanhoEixoZ = value;
                posicaoPontaSetaEixoZ.Y = posicaoPontaSetaEixoZOriginal.Y + (tamanhoEixoZ - tamanhoEixoZOriginal);
            }
        }
        /// <summary>
        /// Posição da letra X em relação à origem do referencial cartesiano
        /// </summary>
        public Vector3 posicaoLetraX;
        /// <summary>
        /// Posição da letra Y em relação à origem do referencial cartesiano
        /// </summary>
        public Vector3 posicaoLetraY;
        /// <summary>
        /// Posição da letra Z em relação à origem do referencial cartesiano
        /// </summary>
        public Vector3 posicaoLetraZ;
        /// <summary>
        /// Posição da ponta da seta do eixo X
        /// </summary>
        public Vector3 posicaoPontaSetaEixoX;
        /// <summary>
        /// Posição da ponta da seta do eixo Y
        /// </summary>
        public Vector3 posicaoPontaSetaEixoY;
        /// <summary>
        /// Posição da ponta da seta do eixo Z
        /// </summary>
        public Vector3 posicaoPontaSetaEixoZ;
        /// <summary>
        /// Posição original da ponta da seta do eixo X
        /// </summary>
        private Vector3 posicaoPontaSetaEixoXOriginal = new Vector3(0.0f, 0.0f, 11.0f);
        /// <summary>
        /// Posição original da ponta da seta do eixo Y
        /// </summary>
        private Vector3 posicaoPontaSetaEixoYOriginal = new Vector3(-11.0f, 0.0f, 0.0f);
        /// <summary>
        /// Posição original da ponta da seta do eixo Z
        /// </summary>
        private Vector3 posicaoPontaSetaEixoZOriginal = new Vector3(0.0f, -11.0f, 0.0f);
        /// <summary>
        /// Tamanho, da base até a ponta, da seta do eixo X
        /// </summary>
        public float tamanhoPontaSetaEixoX = 1.0f;
        /// <summary>
        /// Tamanho, da base até a ponta, da seta do eixo Y
        /// </summary>
        public float tamanhoPontaSetaEixoY = 1.0f;
        /// <summary>
        /// Tamanho, da base até a ponta, da seta do eixo Z
        /// </summary>
        public float tamanhoPontaSetaEixoZ = 1.0f;
        /// <summary>
        /// Tamanho original do eixo X, sem a ponta da seta
        /// </summary>
        public float tamanhoEixoXOriginal = 10.0f;
        /// <summary>
        /// Tamanho original do eixo Y, sem a ponta da seta
        /// </summary>
        public float tamanhoEixoYOriginal = 10.0f;
        /// <summary>
        /// Tamanho original do eixo Z, sem a ponta da seta
        /// </summary>
        public float tamanhoEixoZOriginal = 10.0f;
        /// <summary>
        /// Posição original da letra X
        /// </summary>
        public Vector3 posicaoLetraXOriginal = new Vector3(0.0f, 0.0f, -11.5f);
        /// <summary>
        /// Posição original da letra Y
        /// </summary>
        public Vector3 posicaoLetraYOriginal = new Vector3(-11.5f, 0.0f, 0.0f);
        /// <summary>
        /// Posição original da letra Z
        /// </summary>
        public Vector3 posicaoLetraZOriginal = new Vector3(0.0f, 11.8f, 0.0f);

        /// <summary>
        /// Variável para indicar se o referencial cartesiano é visível.
        /// </summary>
        public bool visivel = false;

        /// <summary>
        /// Construtor padrão da classe ReferencialCartesiano
        /// </summary>
        public ReferencialCartesiano()
        {
            tamanhoEixoX = tamanhoEixoXOriginal;
            tamanhoEixoY = tamanhoEixoYOriginal;
            tamanhoEixoZ = tamanhoEixoZOriginal;

            posicaoPontaSetaEixoX = posicaoPontaSetaEixoXOriginal;
            posicaoPontaSetaEixoY = posicaoPontaSetaEixoYOriginal;
            posicaoPontaSetaEixoZ = posicaoPontaSetaEixoZOriginal;

            posicaoLetraX = posicaoLetraXOriginal;
            posicaoLetraY = posicaoLetraYOriginal;
            posicaoLetraZ = posicaoLetraZOriginal;
        }

        /// <summary>
        /// Matriz de posicionamento da letra X em relação à sua posição padrão
        /// </summary>
        /// <returns>Matriz de posicionamento da letra X</returns>
        public Matrix MatrizPosLetraX()
        {
            Vector3 difPos = posicaoLetraX - posicaoLetraXOriginal;
            return Matrix.CreateTranslation(difPos);
        }

        /// <summary>
        /// Matriz de posicionamento da letra Y em relação à sua posição padrão
        /// </summary>
        /// <returns>Matriz de posicionamento da letra Y</returns>
        public Matrix MatrizPosLetraY()
        {
            Vector3 difPos = posicaoLetraY - posicaoLetraYOriginal;
            return Matrix.CreateTranslation(difPos);
        }

        /// <summary>
        /// Matriz de posicionamento da letra Z em relação à sua posição padrão
        /// </summary>
        /// <returns>Matriz de posicionamento da letra Z</returns>
        public Matrix MatrizPosLetraZ()
        {
            Vector3 difPos = posicaoLetraZ - posicaoLetraZOriginal;
            return Matrix.CreateTranslation(difPos);
        }

        /// <summary>
        /// Matriz de escalonamento (dimensionamento) do eixo X
        /// </summary>
        /// <returns>Matriz de escalonamento (dimensionamento) do eixo X</returns>
        public Matrix MatrizTamanhoEixoX()
        {
            float razaoTam = tamanhoEixoX / tamanhoEixoXOriginal;
            return Matrix.CreateScale(1.0f, 1.0f, razaoTam);
        }

        /// <summary>
        /// Matriz de escalonamento (dimensionamento) do eixo Y
        /// </summary>
        /// <returns>Matriz de escalonamento (dimensionamento) do eixo Y</returns>
        public Matrix MatrizTamanhoEixoY()
        {
            float razaoTam = tamanhoEixoY / tamanhoEixoYOriginal;
            return Matrix.CreateScale(razaoTam, 1.0f, 1.0f);
        }

        /// <summary>
        /// Matriz de escalonamento (dimensionamento) do eixo Z
        /// </summary>
        /// <returns>Matriz de escalonamento (dimensionamento) do eixo Z</returns>
        public Matrix MatrizTamanhoEixoZ()
        {
            float razaoTam = tamanhoEixoZ / tamanhoEixoZOriginal;
            return Matrix.CreateScale(1.0f, razaoTam, 1.0f);
        }

        /// <summary>
        /// Matriz de posição da ponta do eixo X
        /// </summary>
        /// <returns>Matriz de posição da ponta do eixo X</returns>
        public Matrix MatrizPosicaoPontaEixoX()
        {
            Vector3 difPos = posicaoPontaSetaEixoX - posicaoPontaSetaEixoXOriginal;
            return Matrix.CreateTranslation(difPos);
        }

        /// <summary>
        /// Matriz de posição da ponta do eixo Y
        /// </summary>
        /// <returns>Matriz de posição da ponta do eixo Y</returns>
        public Matrix MatrizPosicaoPontaEixoY()
        {
            Vector3 difPos = posicaoPontaSetaEixoY - posicaoPontaSetaEixoYOriginal;
            return Matrix.CreateTranslation(difPos);
        }

        /// <summary>
        /// Matriz de posição da ponta do eixo Z
        /// </summary>
        /// <returns>Matriz de posição da ponta do eixo Z</returns>
        public Matrix MatrizPosicaoPontaEixoZ()
        {
            Vector3 difPos = posicaoPontaSetaEixoZ - posicaoPontaSetaEixoZOriginal;
            return Matrix.CreateTranslation(difPos);
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
        /// Cria uma matriz de rotação a partir de 3 vetores quaisquer. Recomenda-se que eles sejam
        /// perpendiculares entre si.
        /// </summary>
        /// <param name="eixoX">Vetor correspondente ao eixo X</param>
        /// <param name="eixoY">Vetor correspondente ao eixo Y</param>
        /// <param name="eixoZ">Vetor correspondente ao eixo Z</param>
        /// <returns>Uma matriz de rotação correspondente aos 3 vetores passados como parâmetro</returns>
        private Matrix VetoresParaMatrizRotacao(Vector3 eixoX, Vector3 eixoY, Vector3 eixoZ)
        {
            Matrix R = Matrix.Identity;

            eixoX.Normalize();
            eixoY.Normalize();
            eixoZ.Normalize();

            R.M11 = eixoX.X;
            R.M12 = eixoX.Y;
            R.M13 = eixoX.Z;

            R.M21 = eixoY.X;
            R.M22 = eixoY.Y;
            R.M23 = eixoY.Z;

            R.M31 = eixoZ.X;
            R.M32 = eixoZ.Y;
            R.M33 = eixoZ.Z;

            return R;
        }

        /// <summary>
        /// Método para calcular a matriz de rotação correspondente ao vetor obtido a partir da posição da câmera
        /// e da posição do referencial cartesiano em relação à origem da base fixa. O vetor da câmera é um vetor que
        /// aponta para frente (entrando na tela/janela)
        /// </summary>
        /// <param name="posicaoCamera">Posição da câmera em relação à origem da base fixa</param>
        /// <param name="world">Matriz correspondente à posição e orientação do referencial cartesiano em relação à origem da base fixa</param>
        /// <returns>Matriz de rotação referente ao vetor da câmera</returns>
        private Matrix CalculaMatrizRotacaoCamera(Vector3 posicaoCamera, Matrix world)
        {
            Vector3 posReferencial = world.Translation;

            Vector3 vetorCamera = posReferencial - posicaoCamera;

            Vector3 vetorProjCameraXZ = new Vector3(vetorCamera.X, 0.0f, vetorCamera.Z);

            Vector3 vetorPerpProjCameraX = Vector3.Transform(vetorProjCameraXZ, Matrix.CreateRotationY(MathHelper.ToRadians(90.0f)));

            Vector3 vetorPerpProjCameraY = Vector3.Cross(vetorCamera, vetorPerpProjCameraX);

            Matrix rotVetorCam = VetoresParaMatrizRotacao(vetorPerpProjCameraX, vetorPerpProjCameraY, vetorCamera);

            return rotVetorCam;
        }


        /// <summary>
        /// Calcula a matriz de rotação que irá orientar a letra do eixo cartesiano.
        /// </summary>
        /// <param name="world">Matriz correspondente à posição e orientação do referencial cartesiano em relação à origem da base fixa</param>
        /// <param name="rotVetorCam">Matriz de rotação referente ao vetor da câmera</param>
        /// <param name="posLetra">Vetor de posição da letra em relação à origem do referencial cartesiano</param>
        /// <returns>Matriz que irá rotacionar a letra em torno dos eixos X, Y e Z.</returns>
        private Matrix CalculaMatrizRotacaoLetra(Matrix world, Matrix rotVetorCam, Vector3 posLetra)
        {   
            Matrix rotLetra = ExtraiMatrizRotacao(world);
            
            Matrix rotEfetiva = rotVetorCam * Matrix.Invert(rotLetra);
            
            Matrix rotacaoFinal = Matrix.CreateTranslation(-posLetra) * rotEfetiva * Matrix.CreateTranslation(posLetra);

            return rotacaoFinal;
        }

        /// <summary>
        /// Lista de todas as matrizes do modelo 3D do referencial cartesiano
        /// </summary>
        /// <returns>Lista de todas as matrizes do modelo 3D do referencial cartesiano</returns>
        public List<Matrix> Matrizes(Vector3 posicaoCamera, Matrix world)
        {
            List<Matrix> matrizes = new List<Matrix>();
            
            matrizes.Add(MatrizTamanhoEixoX());
            matrizes.Add(MatrizTamanhoEixoY());
            matrizes.Add(MatrizTamanhoEixoZ());

            matrizes.Add(MatrizPosicaoPontaEixoX());
            matrizes.Add(MatrizPosicaoPontaEixoY());
            matrizes.Add(MatrizPosicaoPontaEixoZ());

            Matrix rotVetorCam = CalculaMatrizRotacaoCamera(posicaoCamera, world);

            Matrix matrizLetraX = CalculaMatrizRotacaoLetra(world, rotVetorCam, posicaoLetraX);
            Matrix matrizLetraY = CalculaMatrizRotacaoLetra(world, rotVetorCam, posicaoLetraY);
            Matrix matrizLetraZ = CalculaMatrizRotacaoLetra(world, rotVetorCam, posicaoLetraZ);

            matrizes.Add(MatrizPosLetraX() * matrizLetraX);
            matrizes.Add(MatrizPosLetraY() * matrizLetraY);
            matrizes.Add(MatrizPosLetraZ() * matrizLetraZ);

            return matrizes;
        }
    }
}
