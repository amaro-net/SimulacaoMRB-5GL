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
using Microsoft.Xna.Framework.Graphics;

namespace BracoRobo.Classes
{
    /// <summary>
    /// Classe para representar todos os componentes físicos do robô.
    /// Um objeto ser um ComponenteFisico significa que ele terá todos os
    /// atributos de um componente físico, tais como massa (peso), posição (do centro de massa),
    /// velocidade escalar (translação), ângulo que girou (em torno de x, y e/ou z) e velocidade angular (giro).
    /// Neste caso, objetos que possuem componentes físicos podem (e devem) ser tratados, também, como componentes físicos.
    /// Vale notar, também, que todo componente físico terá seu referencial cartesiano próprio. Ou seja, qualquer componente físico
    /// que componha um outro componente físico terá sua posição especificada em relação a este referencial.
    /// </summary>
    public abstract class ComponenteFisico
    {
        /**** Atributos de um componente físico ****/
        /// <summary>
        /// modelo 3D do componente físico.
        /// </summary>
        public Model modelo;

        // Atributos estáticos
        /// <summary>
        /// Massa do componente físico
        /// </summary>
        public double massa;
        /// <summary>
        /// Angulo de giro em torno de X, de Y e de Z respectivamente
        /// </summary>
        public Vector3 anguloGiro;
        /// <summary>
        /// Posição do centro de massa.
        /// </summary>
        public Vector3 centroMassa;
        /// <summary>
        /// Ponto considerado como sendo a posição do componente físico. Posição considerada para transladar o objeto.
        /// </summary>
        public Vector3 posReferencial;
        /// <summary>
        ///  Posição que será considerada o eixo de giro do componente físico.
        /// </summary>
        public Vector3 centroGiro;

        // Atributos dinâmicos
        /// <summary>
        /// Velocidade nos eixos X, Y e Z.
        /// </summary>
        public Vector3 velocidadeTrans;
        /// <summary>
        /// Velocidade angular em torno de X, de Y e de Z.
        /// </summary>
        public Vector3 velocidadeAng;
        /// <summary>
        /// Aceleração nos eixos X, Y e Z.
        /// </summary>
        public Vector3 aceleracaoTrans;
        /// <summary>
        /// Aceleração angular nos eixos X, Y e Z.
        /// </summary>
        public Vector3 aceleracaoAng;
        
        /// <summary>
        /// Construtor padrão.
        /// </summary>
        public ComponenteFisico()
        {

        }        

        /// <summary>
        /// Função virtual para carregar todo o conteúdo referente ao componente físico
        /// </summary>
        public virtual void LoadContent()
        {
        }

        /// <summary>
        /// Faz o componente físico se mover à velocidadeTrans.
        /// Em outras palavras, faz a posição referencial do objeto (posReferencial) mudar de posição à velocidade contida em velocidadeTrans.
        /// </summary>
        public void Mover()
        {
            posReferencial += velocidadeTrans;
        }
        /// <summary>
        /// Faz o objeto se mover à -velocidadeTrans.
        /// Enquanto o método Mover() faz o objeto se mover em um sentido, MoverInvertido() faz o objeto se mover no sentido contrário.
        /// </summary>
        public void MoverInvertido()
        {
            posReferencial -= velocidadeTrans;
        }
        /// <summary>
        /// Gira o objeto.
        /// Em outras palavras, faz os ângulos do objeto (contidos em anguloGiro) mudarem à velocidade contida em velocidadeAng.
        /// </summary>
        public virtual void Girar()
        {
            this.anguloGiro += this.velocidadeAng;
        }
        /// <summary>
        /// Gira o objeto em sentido contrário ao método Girar().
        /// </summary>
        public virtual void GirarInvertido()
        {
            this.anguloGiro -= this.velocidadeAng;
        }
        /// <summary>
        /// Aumenta a velocidade de translação à taxa contida em aceleracaoTrans.
        /// </summary>
        public void AceleraTrans()
        {
            velocidadeTrans += aceleracaoTrans;
        }
        /// <summary>
        /// Diminui a velocidade de translação à taxa contida em aceleracaoTrans.
        /// </summary>
        public void DesaceleraTrans()
        {
            velocidadeTrans += aceleracaoTrans;
        }
        /// <summary>
        /// Aumenta a velocidade de giro à taxa contida em aceleracaoAng.
        /// </summary>
        public void AceleraAng()
        {
            velocidadeAng += aceleracaoAng;
        }
        /// <summary>
        /// Diminui a velocidade de giro à taxa contida em aceleracaoAng.
        /// </summary>
        public void DesaceleraAng()
        {
            velocidadeAng -= aceleracaoAng;
        }

        /// <summary>
        /// Cria uma matriz de rotação em torno de um eixo paralelo ao eixo X que fica localizado na posição contida em centroGiro.
        /// Em outras palavras, o objeto irá girar em torno de uma reta que está na posição centroGiro e se estende
        /// paralelamente ao eixo X. Usada para modelos 3D sólidos, ou seja, que só possua um mesh.
        /// </summary>
        /// <returns>Retorna uma Matriz de rotação em X para o componente físico</returns>
        public Matrix MatrizRotacaoX()
        {
            return Matrix.CreateRotationX(MathHelper.ToRadians(anguloGiro.X));
        }

        /// <summary>
        /// Cria uma matriz de rotação em torno de um eixo paralelo ao eixo Y que fica localizado na posição contida em centroGiro.
        /// Em outras palavras, o objeto irá girar em torno de uma reta que está na posição centroGiro e se estende
        /// paralelamente ao eixo Y. Usada para modelos 3D sólidos, ou seja, que só possua um mesh.
        /// </summary>
        /// <returns>Retorna uma Matriz de rotação em Y para o componente físico</returns>
        public Matrix MatrizRotacaoY()
        {
            return Matrix.CreateRotationY(MathHelper.ToRadians(anguloGiro.Y));
        }

        /// <summary>
        /// Cria uma matriz de rotação em torno de um eixo paralelo ao eixo Z que fica localizado na posição contida em centroGiro.
        /// Em outras palavras, o objeto irá girar em torno de uma reta que está na posição centroGiro e se estende
        /// paralelamente ao eixo Z. Usada para modelos 3D sólidos, ou seja, que só possua um mesh.
        /// </summary>
        /// <returns>Retorna uma Matriz de rotação em Z para o componente físico</returns>
        public Matrix MatrizRotacaoZ()
        {
            return Matrix.CreateRotationZ(MathHelper.ToRadians(anguloGiro.Z));
        }


        /// <summary>
        /// Cria uma matriz de rotação em torno de um eixo paralelo ao eixo X que fica localizado na posição contida em centroGiro.
        /// Em outras palavras, o objeto irá girar em torno de uma reta que está na posição centroGiro e se estende
        /// paralelamente ao eixo X. Esta matriz representa uma ida do centro de giro à origem, uma rotação em torno de X 
        /// e uma volta ao centro de giro. Usada em modelos 3D que possuam mais de um mesh.
        /// </summary>
        /// <returns>Retorna uma Matriz de rotação em X para o componente físico, com ida à origem, rotação em torno de X, e volta ao centro de giro</returns>
        public Matrix MatrizRotacaoX_ve()
        {
            return Matrix.CreateTranslation(-centroGiro) // leva objeto para o ponto de origem
                   * Matrix.CreateRotationX(MathHelper.ToRadians(anguloGiro.X)) // gira em torno do eixo X
                   * Matrix.CreateTranslation(centroGiro); // retorna o objeto para o ponto onde ele estava
        }
        /// <summary>
        /// Cria uma matriz de rotação em torno de um eixo paralelo ao eixo Y que fica localizado na posição contida em centroGiro.
        /// Em outras palavras, o objeto irá girar em torno de uma reta que está na posição centroGiro e se estende
        /// paralelamente ao eixo Y. Esta matriz representa uma ida do centro de giro à origem, uma rotação em torno de Y 
        /// e uma volta ao centro de giro. Usada em modelos 3D que possuam mais de um mesh.
        /// </summary>
        /// <returns>Retorna uma Matriz de rotação em Y para o componente físico, com ida à origem, rotação em torno de Y, e volta ao centro de giro</returns>
        public Matrix MatrizRotacaoY_ve()
        {
            return Matrix.CreateTranslation(-centroGiro) // leva o objeto para o ponto de origem
                   * Matrix.CreateRotationY(MathHelper.ToRadians(anguloGiro.Y)) // gira em torno do eixo Y
                   * Matrix.CreateTranslation(centroGiro); // retorna o objeto para o ponto onde ele estava
        }
        /// <summary>
        /// Cria uma matriz de rotação em torno de um eixo paralelo ao eixo Z que fica localizado na posição contida em centroGiro.
        /// Em outras palavras, o objeto irá girar em torno de uma reta que está na posição centroGiro e se estende
        /// paralelamente ao eixo Z. Esta matriz representa uma ida do centro de giro à origem, uma rotação em torno de Z 
        /// e uma volta ao centro de giro. Usada em modelos 3D que possuam mais de um mesh.
        /// </summary>
        /// <returns>Retorna uma Matriz de rotação em Z para o componente físico, com ida à origem, rotação em torno de Z, e volta ao centro de giro</returns>
        public Matrix MatrizRotacaoZ_ve()
        {
            return Matrix.CreateTranslation(-centroGiro) // leva o objeto para o ponto de origem
                   * Matrix.CreateRotationZ(MathHelper.ToRadians(anguloGiro.Z)) // gira em torno do eixo Z
                   * Matrix.CreateTranslation(centroGiro);// retorna o objeto para o ponto onde ele estava
        }
        /// <summary>
        /// Cria uma matriz de translação para o componente físico.
        /// </summary>
        /// <returns>Retorna uma Matriz de translação para o componente físico</returns>
        public Matrix MatrizTranslacao()
        {
            return Matrix.CreateTranslation(posReferencial);
        }

        /// <summary>
        /// Método para calcular o momento de inércia de um componente físico.
        /// </summary>
        /// <returns>valor correspondente ao momento de inércia</returns>
        public abstract float MomentoInercia();
    }
}
