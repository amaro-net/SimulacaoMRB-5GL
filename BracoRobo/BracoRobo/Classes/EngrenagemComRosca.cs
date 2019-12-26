﻿/*
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
namespace BracoRobo.Classes
{
    /// <summary>
    /// Classe para representar uma engrenagem com rosca.
    /// </summary>
    public class EngrenagemComRosca : Engrenagem
    {
        /// <summary>
        /// Distância, em milímetros, que a rosca se deslocaria se fosse enroscada 1 volta.
        /// </summary>
        /// <remarks><br></br>
        ///       passoRosca <br></br>
        ///      |---------| <br></br>
        ///  ----/---------/---------/---------/---------/- <br></br>
        ///     /         /         /         /         /   <br></br>
        ///    /         /         /         /         /    <br></br>
        ///   /         /         /         /         /     <br></br>
        /// -/---------/---------/---------/---------/----- <br></br>
        /// </remarks>
        public float passoRosca;
        /// <summary>
        /// Raio do eixo da rosca (sem contar a espessura da rosca).
        /// </summary>
        public float raioEixoEspiral;

        /// <summary>
        /// Construtor padrão
        /// </summary>
        public EngrenagemComRosca()
        {
        }
    }
}
