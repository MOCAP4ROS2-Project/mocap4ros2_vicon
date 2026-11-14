
//////////////////////////////////////////////////////////////////////////////////
// MIT License
//
// Copyright (c) 2020 Vicon Motion Systems Ltd
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <cstdint>
#include <limits>

namespace ClientUtils
{
  /// Generic exponent compile-time calculations. Pow<2,3>::result == 2^3
  constexpr std::uint64_t Pow(const int i_Base, const int i_Exponent)
  {
    return i_Exponent == 0 ? 1 : i_Base * Pow(i_Base, i_Exponent - 1);
  }

  /// Calculate pi to a given number of decimal places using the Bailey-Borwein-Plouffe formula.
  constexpr double CalculatePi(const int i_IterationCount = std::numeric_limits<double>::digits10)
  {
    if (i_IterationCount == -1)
    {
      return 0;
    }
    // Bailey-Borwein-Plouffe formula for calculating pi
    // http://en.wikipedia.org/wiki/Bailey-Borwein-Plouffe_formula
    return i_IterationCount == -1 ? 0 :
                                    (
                                      1.0 / Pow(16, i_IterationCount) *
                                      (4.0 / (8 * i_IterationCount + 1.0) -
                                       2.0 / (8 * i_IterationCount + 4.0) -
                                       1.0 / (8 * i_IterationCount + 5.0) -
                                       1.0 / (8 * i_IterationCount + 6.0))) +
                                      CalculatePi(i_IterationCount - 1);
  }

  /// The mathematical constant Pi.
  constexpr auto Pi = CalculatePi();

  /// Half of the mathematical constant Pi.
  constexpr auto HalfPi = Pi / 2.0;
}
