/*
    Simula��o do Bra�o Rob� MRB-5GL - Simulates MRB-5GL, a 5 DOF Robot Arm prototype

    Copyright (C) 2019  Amaro Duarte de Paula Neto

    This file is part of Simula��o do Bra�o Rob� MRB-5GL.

    Simula��o do Bra�o Rob� MRB-5GL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Simula��o do Bra�o Rob� MRB-5GL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Simula��o do Bra�o Rob� MRB-5GL.  If not, see <https://www.gnu.org/licenses/>.

    contact e-mail: amaro.net80@gmail.com


    Este arquivo � parte do programa Simula��o do Bra�o Rob� MRB-5GL

    Simula��o do Bra�o Rob� MRB-5GL � um software livre; voc� pode redistribu�-lo e/ou
    modific�-lo dentro dos termos da Licen�a P�blica Geral GNU como
    publicada pela Free Software Foundation (FSF); na vers�o 3 da
    Licen�a, ou (a seu crit�rio) qualquer vers�o posterior.

    Simula��o do Bra�o Rob� MRB-5GL � distribu�do na esperan�a de que possa ser �til,
    mas SEM NENHUMA GARANTIA; sem uma garantia impl�cita de ADEQUA��O
    a qualquer MERCADO ou APLICA��O EM PARTICULAR. Veja a
    Licen�a P�blica Geral GNU para maiores detalhes.

    Voc� deve ter recebido uma c�pia da Licen�a P�blica Geral GNU junto
    com este programa, Se n�o, veja <http://www.gnu.org/licenses/>.
*/
using System;

namespace BracoRobo
{
#if WINDOWS || XBOX
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        static void Main(string[] args)
        {
            using (SimulacaoDoRobo simu = new SimulacaoDoRobo(args))
            {
                simu.Run();
            }
        }
    }
#endif
}

