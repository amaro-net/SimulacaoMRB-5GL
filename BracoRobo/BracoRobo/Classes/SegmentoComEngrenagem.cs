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
    /// Classe que representa um segmento com engrenagem
    /// </summary>
    public class SegmentoComEngrenagem : ComponenteFisico
    {
        /// <summary>
        /// Tamanho do segmento entre os furos dos parafusos
        /// </summary>
        public float tamanhoSegmento;
        /// <summary>
        /// Quantidade de dentes que a parte engrenada teria se fosse dada uma volta de 360 graus
        /// </summary>
        public float numDentes;
        /// <summary>
        /// Raio da parte do segmento que possui a engrenagem. Este valor inclui a espessura dos dentes.
        /// </summary>
        public float raioEngrenagem;
        /// <summary>
        /// Espessura dos dentes da parte do segmento que possui engrenagem.
        /// </summary>
        public float espessuraDosDentes;

        /// <summary>
        /// Construtor da classe segmento com engrenagem. Inicia os campos tamanhoSegmento, 
        /// numDentes, raioEngrenagem e espessuraDosDentes com valores padrão.
        /// </summary>
        public SegmentoComEngrenagem()
        {
            this.tamanhoSegmento = 3.1f;
            this.numDentes = 18f;
            this.raioEngrenagem = 1.5f;
            this.espessuraDosDentes = 0.35f;
        }
        /// <summary>
        /// Não implementado
        /// </summary>
        /// <returns></returns>
        public override float MomentoInercia()
        {
            throw new NotImplementedException();
        }
    }
}
