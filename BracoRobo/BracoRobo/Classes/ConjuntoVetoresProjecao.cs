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
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;
using System.Text.RegularExpressions;

namespace BracoRobo.Classes
{
    /// <summary>
    /// Classe que representa o conjunto de 3 vetores obtidos na projeção da posição alvo do braço robô na cinemática inversa
    /// </summary>
    public class ConjuntoVetoresProjecao
    {
        /// <summary>
        /// Modelo 3D contendo os vetores K, M e Ztl
        /// </summary>
        public Model modeloFixo;

        /// <summary>
        /// Modelo 3D do vetor Zt
        /// </summary>
        public Model modeloVetorZt;

        /// <summary>
        /// Tamanho do vetor K, sem a ponta da seta
        /// </summary>
        private float tamanhoVetorK;
        /// <summary>
        /// Tamanho do vetor K, sem a ponta da seta (property)
        /// </summary>
        public float TamanhoVetorK
        {
            get { return tamanhoVetorK; }
            set 
            {
                tamanhoVetorK = value;
                posicaoPontaSetaVetorK.Z = posicaoPontaSetaVetorKOriginal.Z - (tamanhoVetorK - tamanhoVetorKOriginal);
            }
        }
        /// <summary>
        /// Tamanho do vetor M, sem a ponta da seta
        /// </summary>
        private float tamanhoVetorM;
        /// <summary>
        /// Tamanho do vetor M, sem a ponta da seta (property)
        /// </summary>
        public float TamanhoVetorM
        {
            get { return tamanhoVetorM; }
            set 
            { 
                tamanhoVetorM = value;
                posicaoPontaSetaVetorM.X = posicaoPontaSetaVetorMOriginal.X - (tamanhoVetorM - tamanhoVetorMOriginal);
            }
        }
        /// <summary>
        /// Tamanho do vetor Ztl, sem a ponta da seta
        /// </summary>
        private float tamanhoVetorZtl;
        /// <summary>
        /// Tamanho do Vetor Ztl, sem a ponta da seta (property)
        /// </summary>
        public float TamanhoVetorZtl
        {
            get { return tamanhoVetorZtl; }
            set 
            {
                tamanhoVetorZtl = value;
                posicaoPontaSetaVetorZtl.Y = posicaoPontaSetaVetorZtlOriginal.Y + (tamanhoVetorZtl - tamanhoVetorZtlOriginal);
            }
        }
        /// <summary>
        /// Tamanho do vetor Zt, sem a ponta da seta
        /// </summary>
        private float tamanhoVetorZt;
        /// <summary>
        /// Tamanho do vetor Zt, sem a ponta da seta (property)
        /// </summary>
        public float TamanhoVetorZt
        {
            get { return tamanhoVetorZt; }
            set 
            {
                tamanhoVetorZt = value;
                posicaoPontaSetaVetorZt.Y = posicaoPontaSetaVetorZtOriginal.Y + (tamanhoVetorZt - tamanhoVetorZtOriginal);
            }
        }        
        /// <summary>
        /// Posição da letra K em relação ao ponto comum dos vetores
        /// </summary>
        public Vector3 posicaoLetraK;
        /// <summary>
        /// Posição da letra M em relação ao ponto comum dos vetores
        /// </summary>
        public Vector3 posicaoLetraM;
        /// <summary>
        /// Posição da letra Ztl em relação ao ponto comum dos vetores
        /// </summary>
        public Vector3 posicaoLetraZtl;
        /// <summary>
        /// Posicao da letra Zt em relacao ao ponto comum dos vetores
        /// </summary>
        public Vector3 posicaoLetraZt;
        /// <summary>
        /// Posição da ponta da seta do vetor K
        /// </summary>
        public Vector3 posicaoPontaSetaVetorK;
        /// <summary>
        /// Posição da ponta da seta do vetor M
        /// </summary>
        public Vector3 posicaoPontaSetaVetorM;
        /// <summary>
        /// Posição da ponta da seta do vetor Ztl
        /// </summary>
        public Vector3 posicaoPontaSetaVetorZtl;
        /// <summary>
        /// Posição da ponta da seta do vetor Zt
        /// </summary>
        public Vector3 posicaoPontaSetaVetorZt;
        /// <summary>
        /// Posição original da ponta da seta do vetor K
        /// </summary>
        private Vector3 posicaoPontaSetaVetorKOriginal = new Vector3(0.0f, 0.0f, 11.0f);
        /// <summary>
        /// Posição original da ponta da seta do vetor M
        /// </summary>
        private Vector3 posicaoPontaSetaVetorMOriginal = new Vector3(-11.0f, 0.0f, 0.0f);
        /// <summary>
        /// Posição original da ponta da seta do vetor Ztl
        /// </summary>
        private Vector3 posicaoPontaSetaVetorZtlOriginal = new Vector3(0.0f, -11.0f, 0.0f);
        /// <summary>
        /// Posição original da ponta da seta do vetor Zt
        /// </summary>
        private Vector3 posicaoPontaSetaVetorZtOriginal = new Vector3(0.0f, -11.0f, 0.0f);        
        /// <summary>
        /// Tamanho, da base até a ponta, da seta do vetor K
        /// </summary>
        public float tamanhoPontaSetaVetorK = 1.0f;
        /// <summary>
        /// Tamanho, da base até a ponta, da seta do vetor M
        /// </summary>
        public float tamanhoPontaSetaVetorM = 1.0f;
        /// <summary>
        /// Tamanho, da base até a ponta, da seta do vetor Ztl
        /// </summary>
        public float tamanhoPontaSetaVetorZtl = 1.0f;
        /// <summary>
        /// Tamanho, da base até a ponta, da seta do vetor Zt
        /// </summary>
        public float tamanhoPontaSetaVetorZt = 1.0f;
        /// <summary>
        /// Tamanho original do vetor K, sem a ponta da seta
        /// </summary>
        public float tamanhoVetorKOriginal = 10.0f;
        /// <summary>
        /// Tamanho original do vetor M, sem a ponta da seta
        /// </summary>
        public float tamanhoVetorMOriginal = 10.0f;
        /// <summary>
        /// Tamanho original do vetor Ztl, sem a ponta da seta
        /// </summary>
        public float tamanhoVetorZtlOriginal = 10.0f;
        /// <summary>
        /// Tamanho original do vetor Zt, sem a ponta da seta
        /// </summary>
        public float tamanhoVetorZtOriginal = 10.0f;
        /// <summary>
        /// Posição original da letra K
        /// </summary>
        public Vector3 posicaoLetraKOriginal = new Vector3(0.0f, 0.0f, -11.5f);
        /// <summary>
        /// Posição original da letra M
        /// </summary>
        public Vector3 posicaoLetraMOriginal = new Vector3(-11.5f, 0.0f, 0.0f);
        /// <summary>
        /// Posição original da letra Ztl
        /// </summary>
        public Vector3 posicaoLetraZtlOriginal = new Vector3(0.0f, 11.8f, 0.0f);
        /// <summary>
        /// Posição original da letra Zt
        /// </summary>
        public Vector3 posicaoLetraZtOriginal = new Vector3(0.0f, 11.8f, 0.0f);
        /// <summary>
        /// Variável para indicar se o conjunto de vetores é visível.
        /// </summary>
        public bool visivel = false;
        /// <summary>
        /// Indica se o vetor M deve ser invertido
        /// </summary>
        public bool inverteVetoresMeK = false;
        /// <summary>
        /// Ângulo teta em graus entre o vetor Zt e o vetor Ztl
        /// </summary>
        public float anguloTetaZtProj = 0f;
        /// <summary>
        /// Ângulo teta em graus que o modelo 3D do Vetor Zt irá rotacionar
        /// </summary>
        public float anguloTetaZtRot = 0f;
        /// <summary>
        /// Campo para indicar se a posição do conjunto de vetores, incluindo o anguloTetaZtProj, será
        /// determinado pelo cálculo da matriz de transformação obtida a partir de uma posição alvo,
        /// sendo esta determinada por coordenadas x, y, z, gama, beta e alfa, sendo estes 3 últimos
        /// valores os ângulos de giro (em ângulos fixos) em torno dos eixos X, Y e Z do referencial 
        /// da base fixa.
        /// </summary>
        public bool determinadoPorPosicaoAlvo = false;
        /// <summary>
        /// Posição XYZ alvo em relação a base fixa do braço robô
        /// </summary>
        public Vector3 xyzAlvo = new Vector3(0f, 0f, 0f);
        /// <summary>
        /// Ângulo alvo de rotação em torno do eixo X da base fixa do braço robô
        /// </summary>
        public float gamaAlvo = 0f;
        /// <summary>
        /// Ângulo alvo de rotação em torno do eixo Y da base fixa do braço robô
        /// </summary>
        public float betaAlvo = 0f;
        /// <summary>
        /// Ângulo alvo de rotação em torno do eixo Z da base fixa do braço robô
        /// </summary>
        public float alfaAlvo = 0f;
        /// <summary>
        /// String que armazena o padrão de expressão regular para reconhecer uma posição alvo digitada no console
        /// </summary>
        private String padraoPosAlvoRegex = @"^\s*[+-]?(\d*(\.\d*)?){1}(\s*\,\s*[+-]?(\d*(\.\d*)?){1}){5}$";
        /// <summary>
        /// Objeto para a expressão regular que reconhece a entrada de posição alvo.
        /// </summary>
        public Regex expressaoRegularPosAlvo;
        /// <summary>
        /// Matriz da posição alvo. Esta matriz posiciona o conjunto de vetores.
        /// </summary>
        public Matrix matrizPosicaoAlvo = Matrix.Identity;
        /// <summary>
        /// Matriz a ser utilizada para calcular a posição alvo deseja da garra, sem a projeção no plano vertical.
        /// </summary>
        public Matrix matrizPosZtGarra = Matrix.Identity;
        /// <summary>
        /// Matriz world que dita a posição do ponto comum do conjunto de vetores em relação a base fixa.
        /// </summary>
        public Matrix world;

        private float L3 = 8.633297f;
        private float Lg = 7.5f;
        private double d2 = 0;
        private double d3 = 0;
        private double d4 = 0;

        /// <summary>
        /// Construtor padrão da classe ConjuntoVetoresProjecao
        /// </summary>
        public ConjuntoVetoresProjecao()
        {
            tamanhoVetorK = tamanhoVetorKOriginal;
            tamanhoVetorM = tamanhoVetorMOriginal;
            tamanhoVetorZtl = tamanhoVetorZtlOriginal;
            tamanhoVetorZt = tamanhoVetorZtOriginal;

            posicaoPontaSetaVetorK = posicaoPontaSetaVetorKOriginal;
            posicaoPontaSetaVetorM = posicaoPontaSetaVetorMOriginal;
            posicaoPontaSetaVetorZtl = posicaoPontaSetaVetorZtlOriginal;
            posicaoPontaSetaVetorZt = posicaoPontaSetaVetorZtOriginal;

            posicaoLetraK = posicaoLetraKOriginal;
            posicaoLetraM = posicaoLetraMOriginal;
            posicaoLetraZtl = posicaoLetraZtlOriginal;
            posicaoLetraZt = posicaoLetraZtOriginal;

            expressaoRegularPosAlvo = new Regex(padraoPosAlvoRegex);
        }

        public void AtualizarMatrizPosicaoAlvo()
        {
            float gama = MathHelper.ToRadians(gamaAlvo);
            float beta = MathHelper.ToRadians(betaAlvo);
            float alfa = MathHelper.ToRadians(alfaAlvo);

            double sgama = Math.Sin(gama);
            double sbeta = Math.Sin(beta);
            double salfa = Math.Sin(alfa);

            double cgama = Math.Cos(gama);
            double cbeta = Math.Cos(beta);
            double calfa = Math.Cos(alfa);

            double r11 = calfa * cbeta;
            double r21 = salfa * cbeta;
            double r31 = -sbeta;

            double r12 = calfa * sbeta * sgama - salfa * cgama;
            double r22 = salfa * sbeta * sgama + calfa * cgama;
            double r32 = cbeta * sgama;
            double r13 = calfa * sbeta * cgama + salfa * sgama;
            double r23 = salfa * sbeta * cgama - calfa * sgama;
            double r33 = cbeta * cgama;

            this.matrizPosZtGarra = Matrix.CreateTranslation(xyzAlvo);
            this.matrizPosZtGarra.M11 = (float)r11;
            this.matrizPosZtGarra.M12 = (float)r21;
            this.matrizPosZtGarra.M13 = (float)r31;
            this.matrizPosZtGarra.M21 = (float)r12;
            this.matrizPosZtGarra.M22 = (float)r22;
            this.matrizPosZtGarra.M23 = (float)r32;
            this.matrizPosZtGarra.M31 = (float)r13;
            this.matrizPosZtGarra.M32 = (float)r23;
            this.matrizPosZtGarra.M33 = (float)r33;

            double d234 = d2 + d3 + d4;
            double d234quad = d234 * d234;

            float teta1max = MathHelper.ToRadians(100);
            float teta1min = MathHelper.ToRadians(-90);

            float teta2max = MathHelper.ToRadians(130);
            float teta2min = MathHelper.ToRadians(0);

            float teta3max = MathHelper.ToRadians(0);
            float teta3min = MathHelper.ToRadians(-133);

            float teta4max = MathHelper.ToRadians(164);
            float teta4min = MathHelper.ToRadians(-36);

            float teta5max = MathHelper.ToRadians(90);
            float teta5min = MathHelper.ToRadians(-90);

            Vector3 Zt = new Vector3((float)r13, (float)r23, (float)r33);
            Vector3 Yt = new Vector3((float)r12, (float)r22, (float)r32);

            Zt.Normalize();
            Yt.Normalize();

            Vector3 posicaoXYZConvertida = new Vector3(-xyzAlvo.Y, xyzAlvo.Z, -xyzAlvo.X);
                        
            Vector3 M;
            if (xyzAlvo.X != 0 || xyzAlvo.Y != 0)
            {
                M = new Vector3(-xyzAlvo.Y, xyzAlvo.X, 0.0f);                
            }
            else
            {
                M = new Vector3((float)-salfa, (float)calfa, 0.0f);
            }

            M.Normalize();

            Vector3 K = Vector3.Cross(M, Zt);
            K.Normalize();
            Vector3 Ztl = Vector3.Cross(K, M);
            Ztl.Normalize();

            float cteta = Vector3.Dot(Zt, Ztl);
            float steta = Vector3.Dot(Vector3.Cross(Zt, Ztl), K);

            this.anguloTetaZtProj = MathHelper.ToDegrees((float)Math.Atan2(steta, cteta));

            Vector3 Ytl = cteta * Yt + steta * Vector3.Cross(K, Yt) + (1 - cteta) * Vector3.Dot(K, Yt) * K;

            Ytl.Normalize();

            Vector3 Xtl = Vector3.Cross(Ytl, Ztl);

            Xtl.Normalize();
            
            r11 = Xtl.X;
            r21 = Xtl.Y;
            r31 = Xtl.Z;
            
            r12 = Ytl.X;
            r22 = Ytl.Y;
            r32 = Ytl.Z;
            
            r13 = Ztl.X;
            r23 = Ztl.Y;
            r33 = Ztl.Z;

            double px = xyzAlvo.X - (L3 + Lg) * r13;
            double py = xyzAlvo.Y - (L3 + Lg) * r23;
            double pz = xyzAlvo.Z - (L3 + Lg) * r33;

            double px2py2 = Math.Pow(px, 2.0) + Math.Pow(py, 2.0);
            double sqrtpx2py2 = Math.Sqrt(px2py2);

            // Cálculo do teta1 (todas as soluções possíveis)
            double atan2pypx = Math.Atan2(-px, py);
            double atan2sqrt = Math.Atan2(Math.Sqrt(px2py2 - d234quad), -d234);

            double teta1_1 = atan2pypx + atan2sqrt;
            double teta1_2 = atan2pypx - atan2sqrt;
            double teta1_3 = Math.Atan2(r23, r13);
            double teta1_4 = Math.Atan2(-r23, -r13);

            double teta1, teta234, c1, s1, c234, s234;

            if (teta1_1 >= teta1min && teta1_1 <= teta1max)
                teta1 = teta1_1;
            else if (teta1_2 >= teta1min && teta1_2 <= teta1max)
                teta1 = teta1_2;
            else if (teta1_3 >= teta1min && teta1_3 <= teta1max)
                teta1 = teta1_3;
            else //if (teta1_4 >= teta1min && teta1_4 <= teta1max)
                teta1 = teta1_4;

            s1 = Math.Sin(teta1);
            c1 = Math.Cos(teta1);

            s234 = c1 * r13 + s1 * r23;
            c234 = -r33;

            teta234 = Math.Atan2(s234, c234);

            float anguloEmRelacaoABaseFixa = (float)(teta234 - MathHelper.PiOver2);

            this.inverteVetoresMeK = ((teta1 < 0 && xyzAlvo.Y < 0) || (teta1 > 0 && xyzAlvo.Y > 0) || (teta1 == 0 && xyzAlvo.X >= 0));
            
            Matrix R = Matrix.CreateRotationY(MathHelper.ToRadians(180f))
                     * Matrix.CreateRotationX(MathHelper.ToRadians(-90f))
                     * Matrix.CreateRotationX(anguloEmRelacaoABaseFixa)
                     * Matrix.CreateRotationY((float)(teta1));

            matrizPosicaoAlvo = R * Matrix.CreateTranslation(posicaoXYZConvertida);
        }

        /// <summary>
        /// Matriz de posicionamento da letra K em relação à sua posição padrão
        /// </summary>
        /// <returns>Matriz de posicionamento da letra K</returns>
        public Matrix MatrizPosLetraK()
        {
            Vector3 difPos = posicaoLetraK - posicaoLetraKOriginal;
            return Matrix.CreateTranslation(difPos);
        }

        /// <summary>
        /// Matriz de posicionamento da letra M em relação à sua posição padrão
        /// </summary>
        /// <returns>Matriz de posicionamento da letra M</returns>
        public Matrix MatrizPosLetraM()
        {
            Vector3 difPos = posicaoLetraM - posicaoLetraMOriginal;
            return Matrix.CreateTranslation(difPos);
        }

        /// <summary>
        /// Matriz de posicionamento da letra Ztl em relação à sua posição padrão
        /// </summary>
        /// <returns>Matriz de posicionamento da letra Ztl</returns>
        public Matrix MatrizPosLetraZtl()
        {
            Vector3 difPos = posicaoLetraZtl - posicaoLetraZtlOriginal;
            return Matrix.CreateTranslation(difPos);
        }

        /// <summary>
        /// Matriz de posicionamento da letra Zt em relação à sua posição padrão
        /// </summary>
        /// <returns>Matriz de posicionamento da letra Zt</returns>
        public Matrix MatrizPosLetraZt()
        {
            Vector3 difPos = posicaoLetraZt - posicaoLetraZtOriginal;
            return Matrix.CreateTranslation(difPos);
        }

        /// <summary>
        /// Matriz de escalonamento (dimensionamento) do vetor K
        /// </summary>
        /// <returns>Matriz de escalonamento (dimensionamento) do vetor K</returns>
        public Matrix MatrizTamanhoVetorK()
        {
            float razaoTam = tamanhoVetorK / tamanhoVetorKOriginal;
            return Matrix.CreateScale(1.0f, 1.0f, razaoTam);
        }

        /// <summary>
        /// Matriz de escalonamento (dimensionamento) do vetor M
        /// </summary>
        /// <returns>Matriz de escalonamento (dimensionamento) do vetor M</returns>
        public Matrix MatrizTamanhoVetorM()
        {
            float razaoTam = tamanhoVetorM / tamanhoVetorMOriginal;
            return Matrix.CreateScale(razaoTam, 1.0f, 1.0f);
        }

        /// <summary>
        /// Matriz de escalonamento (dimensionamento) do vetor Ztl
        /// </summary>
        /// <returns>Matriz de escalonamento (dimensionamento) do vetor Ztl</returns>
        public Matrix MatrizTamanhoVetorZtl()
        {
            float razaoTam = tamanhoVetorZtl / tamanhoVetorZtlOriginal;
            return Matrix.CreateScale(1.0f, razaoTam, 1.0f);
        }

        /// <summary>
        /// Matriz de escalonamento (dimensionamento) do vetor Zt
        /// </summary>
        /// <returns>Matriz de escalonamento (dimensionamento) do vetor Zt</returns>
        public Matrix MatrizTamanhoVetorZt()
        {
            float razaoTam = tamanhoVetorZt / tamanhoVetorZtOriginal;
            return Matrix.CreateScale(1.0f, razaoTam, 1.0f);
        }

        /// <summary>
        /// Matriz de posição da ponta do vetor K
        /// </summary>
        /// <returns>Matriz de posição da ponta do vetor K</returns>
        public Matrix MatrizPosicaoPontaVetorK()
        {
            Vector3 difPos = posicaoPontaSetaVetorK - posicaoPontaSetaVetorKOriginal;
            return Matrix.CreateTranslation(difPos);
        }

        /// <summary>
        /// Matriz de posição da ponta do vetor M
        /// </summary>
        /// <returns>Matriz de posição da ponta do vetor M</returns>
        public Matrix MatrizPosicaoPontaVetorM()
        {
            Vector3 difPos = posicaoPontaSetaVetorM - posicaoPontaSetaVetorMOriginal;
            return Matrix.CreateTranslation(difPos);
        }

        /// <summary>
        /// Matriz de posição da ponta do vetor Ztl
        /// </summary>
        /// <returns>Matriz de posição da ponta do vetor Ztl</returns>
        public Matrix MatrizPosicaoPontaVetorZtl()
        {
            Vector3 difPos = posicaoPontaSetaVetorZtl - posicaoPontaSetaVetorZtlOriginal;
            return Matrix.CreateTranslation(difPos);
        }

        /// <summary>
        /// Matriz de posição da ponta do vetor Zt
        /// </summary>
        /// <returns>Matriz de posição da ponta do vetor Zt</returns>
        public Matrix MatrizPosicaoPontaVetorZt()
        {
            Vector3 difPos = posicaoPontaSetaVetorZt - posicaoPontaSetaVetorZtOriginal;
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
        /// e da posição do conjunto de vetores em relação à origem da base fixa. O vetor da câmera é um vetor que
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
        /// Calcula a matriz de rotação que irá orientar a letra do vetor.
        /// </summary>
        /// <param name="world">Matriz correspondente à posição e orientação do conjunto de vetores em relação à origem da base fixa</param>
        /// <param name="rotVetorCam">Matriz de rotação referente ao vetor da câmera</param>
        /// <param name="posLetra">Vetor de posição da letra em relação ao ponto comum do conjunto de vetores</param>
        /// <returns>Matriz que irá rotacionar a letra em torno dos eixos X, Y e Z.</returns>
        private Matrix CalculaMatrizRotacaoLetra(Matrix world, Matrix rotVetorCam, Vector3 posLetra)
        {   
            Matrix rotLetra = ExtraiMatrizRotacao(world);
            
            Matrix rotEfetiva = rotVetorCam * Matrix.Invert(rotLetra);
            
            Matrix rotacaoFinal = Matrix.CreateTranslation(-posLetra) * rotEfetiva * Matrix.CreateTranslation(posLetra);

            return rotacaoFinal;
        }

        /// <summary>
        /// Lista de todas as matrizes do modelo 3D do conjunto de vetores
        /// </summary>
        /// <returns>Lista de todas as matrizes do modelo 3D do conjunto de vetores</returns>
        public List<Matrix> Matrizes(Vector3 posicaoCamera, Matrix world)
        {
            List<Matrix> matrizes = new List<Matrix>();

            Matrix matrizInversaoVetores = Matrix.Identity;

            if (inverteVetoresMeK)
                matrizInversaoVetores = Matrix.CreateRotationY(MathHelper.ToRadians(180.0f));

            matrizes.Add(MatrizTamanhoVetorK() * matrizInversaoVetores);
            matrizes.Add(MatrizTamanhoVetorM() * matrizInversaoVetores);
            matrizes.Add(MatrizTamanhoVetorZtl());

            matrizes.Add(MatrizPosicaoPontaVetorK() * matrizInversaoVetores);
            matrizes.Add(MatrizPosicaoPontaVetorM() * matrizInversaoVetores);
            matrizes.Add(MatrizPosicaoPontaVetorZtl());

            Matrix rotVetorCam = CalculaMatrizRotacaoCamera(posicaoCamera, world);

            Matrix matrizLetraK = CalculaMatrizRotacaoLetra(matrizInversaoVetores * world, rotVetorCam, posicaoLetraK);
            Matrix matrizLetraM = CalculaMatrizRotacaoLetra(matrizInversaoVetores * world, rotVetorCam, posicaoLetraM);
            Matrix matrizLetraZtl = CalculaMatrizRotacaoLetra(world, rotVetorCam, posicaoLetraZtl);

            matrizes.Add(MatrizPosLetraK() * matrizLetraK * matrizInversaoVetores);
            matrizes.Add(MatrizPosLetraM() * matrizLetraM * matrizInversaoVetores);
            matrizes.Add(MatrizPosLetraZtl() * matrizLetraZtl);

            Matrix matrizAnguloTeta;

            if (determinadoPorPosicaoAlvo && inverteVetoresMeK)
            {
                this.anguloTetaZtRot = this.anguloTetaZtProj;
                matrizAnguloTeta = Matrix.CreateRotationZ(MathHelper.ToRadians(this.anguloTetaZtRot))
                                 * matrizInversaoVetores;
            }
            else
            {
                if (!determinadoPorPosicaoAlvo && inverteVetoresMeK)
                {
                    this.anguloTetaZtProj = -this.anguloTetaZtRot;
                    this.matrizPosZtGarra = Matrix.CreateRotationZ(MathHelper.ToRadians(this.anguloTetaZtProj)) * world;
                }
                else
                {
                    if (!determinadoPorPosicaoAlvo)
                    {
                        this.anguloTetaZtProj = this.anguloTetaZtRot;
                        this.matrizPosZtGarra = Matrix.CreateRotationZ(MathHelper.ToRadians(-this.anguloTetaZtProj)) * world;
                    }
                    else
                        this.anguloTetaZtRot = this.anguloTetaZtProj;
                }

                matrizAnguloTeta = Matrix.CreateRotationZ(MathHelper.ToRadians(this.anguloTetaZtRot));                
            }           

            matrizes.Add(MatrizTamanhoVetorZt() * matrizAnguloTeta);
            matrizes.Add(MatrizPosicaoPontaVetorZt() * matrizAnguloTeta);

            Matrix matrizLetraZt = CalculaMatrizRotacaoLetra(matrizAnguloTeta * world, rotVetorCam, posicaoLetraZt);

            matrizes.Add(MatrizPosLetraZt() * matrizLetraZt * matrizAnguloTeta);

            return matrizes;
        }
    }
}
