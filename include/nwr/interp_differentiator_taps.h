/*
 * Copyright (C) 2017  Andy Walls <awalls.cx18@gmail.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this file; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
/*
 * This file was machine generated by gen_interp_differentiator_taps.
 * DO NOT EDIT BY HAND.
 */

static const int	DNTAPS     =    8;
static const int	DNSTEPS    =  128;
static const double	DBANDWIDTH = 0.25;

static const float Dtaps[DNSTEPS+1][DNTAPS] = {
  //    -4            -3            -2            -1             0             1             2             3                mu
  { -1.97975e-02,  1.09180e-01, -3.53697e-01,  1.00394e+00, -1.76425e-01, -6.98940e-01,  1.60378e-01, -2.55693e-02 }, //   0/128
  { -1.98041e-02,  1.09378e-01, -3.55314e-01,  1.01662e+00, -1.98983e-01, -6.86147e-01,  1.58694e-01, -2.53710e-02 }, //   1/128
  { -1.98003e-02,  1.09519e-01, -3.56761e-01,  1.02906e+00, -2.21442e-01, -6.73286e-01,  1.56945e-01, -2.51608e-02 }, //   2/128
  { -1.97862e-02,  1.09604e-01, -3.58039e-01,  1.04126e+00, -2.43798e-01, -6.60361e-01,  1.55133e-01, -2.49389e-02 }, //   3/128
  { -1.97616e-02,  1.09633e-01, -3.59146e-01,  1.05322e+00, -2.66043e-01, -6.47378e-01,  1.53260e-01, -2.47055e-02 }, //   4/128
  { -1.97267e-02,  1.09605e-01, -3.60080e-01,  1.06494e+00, -2.88174e-01, -6.34339e-01,  1.51327e-01, -2.44607e-02 }, //   5/128
  { -1.96814e-02,  1.09520e-01, -3.60840e-01,  1.07640e+00, -3.10183e-01, -6.21250e-01,  1.49335e-01, -2.42049e-02 }, //   6/128
  { -1.96258e-02,  1.09379e-01, -3.61425e-01,  1.08760e+00, -3.32066e-01, -6.08114e-01,  1.47287e-01, -2.39381e-02 }, //   7/128
  { -1.95598e-02,  1.09180e-01, -3.61833e-01,  1.09855e+00, -3.53816e-01, -5.94935e-01,  1.45183e-01, -2.36607e-02 }, //   8/128
  { -1.94834e-02,  1.08925e-01, -3.62064e-01,  1.10923e+00, -3.75429e-01, -5.81717e-01,  1.43025e-01, -2.33727e-02 }, //   9/128
  { -1.93968e-02,  1.08612e-01, -3.62117e-01,  1.11965e+00, -3.96899e-01, -5.68465e-01,  1.40814e-01, -2.30745e-02 }, //  10/128
  { -1.92998e-02,  1.08243e-01, -3.61990e-01,  1.12979e+00, -4.18221e-01, -5.55183e-01,  1.38553e-01, -2.27662e-02 }, //  11/128
  { -1.91927e-02,  1.07816e-01, -3.61683e-01,  1.13966e+00, -4.39388e-01, -5.41875e-01,  1.36242e-01, -2.24481e-02 }, //  12/128
  { -1.90753e-02,  1.07333e-01, -3.61195e-01,  1.14926e+00, -4.60397e-01, -5.28544e-01,  1.33883e-01, -2.21203e-02 }, //  13/128
  { -1.89477e-02,  1.06793e-01, -3.60525e-01,  1.15857e+00, -4.81241e-01, -5.15195e-01,  1.31477e-01, -2.17832e-02 }, //  14/128
  { -1.88101e-02,  1.06195e-01, -3.59672e-01,  1.16761e+00, -5.01915e-01, -5.01832e-01,  1.29027e-01, -2.14369e-02 }, //  15/128
  { -1.86623e-02,  1.05541e-01, -3.58635e-01,  1.17635e+00, -5.22415e-01, -4.88459e-01,  1.26533e-01, -2.10816e-02 }, //  16/128
  { -1.85046e-02,  1.04831e-01, -3.57414e-01,  1.18481e+00, -5.42735e-01, -4.75079e-01,  1.23997e-01, -2.07177e-02 }, //  17/128
  { -1.83369e-02,  1.04064e-01, -3.56008e-01,  1.19297e+00, -5.62871e-01, -4.61698e-01,  1.21421e-01, -2.03452e-02 }, //  18/128
  { -1.81593e-02,  1.03240e-01, -3.54416e-01,  1.20083e+00, -5.82817e-01, -4.48318e-01,  1.18806e-01, -1.99646e-02 }, //  19/128
  { -1.79718e-02,  1.02361e-01, -3.52639e-01,  1.20840e+00, -6.02568e-01, -4.34944e-01,  1.16154e-01, -1.95759e-02 }, //  20/128
  { -1.77746e-02,  1.01425e-01, -3.50676e-01,  1.21567e+00, -6.22120e-01, -4.21579e-01,  1.13466e-01, -1.91794e-02 }, //  21/128
  { -1.75678e-02,  1.00434e-01, -3.48526e-01,  1.22263e+00, -6.41467e-01, -4.08229e-01,  1.10744e-01, -1.87754e-02 }, //  22/128
  { -1.73513e-02,  9.93878e-02, -3.46188e-01,  1.22928e+00, -6.60606e-01, -3.94895e-01,  1.07989e-01, -1.83641e-02 }, //  23/128
  { -1.71253e-02,  9.82862e-02, -3.43664e-01,  1.23563e+00, -6.79531e-01, -3.81583e-01,  1.05203e-01, -1.79458e-02 }, //  24/128
  { -1.68899e-02,  9.71297e-02, -3.40952e-01,  1.24166e+00, -6.98238e-01, -3.68296e-01,  1.02387e-01, -1.75206e-02 }, //  25/128
  { -1.66451e-02,  9.59188e-02, -3.38052e-01,  1.24738e+00, -7.16722e-01, -3.55037e-01,  9.95428e-02, -1.70889e-02 }, //  26/128
  { -1.63911e-02,  9.46536e-02, -3.34965e-01,  1.25278e+00, -7.34980e-01, -3.41811e-01,  9.66721e-02, -1.66509e-02 }, //  27/128
  { -1.61280e-02,  9.33347e-02, -3.31690e-01,  1.25786e+00, -7.53006e-01, -3.28622e-01,  9.37762e-02, -1.62068e-02 }, //  28/128
  { -1.58558e-02,  9.19624e-02, -3.28227e-01,  1.26263e+00, -7.70796e-01, -3.15472e-01,  9.08568e-02, -1.57569e-02 }, //  29/128
  { -1.55747e-02,  9.05371e-02, -3.24577e-01,  1.26707e+00, -7.88347e-01, -3.02366e-01,  8.79152e-02, -1.53014e-02 }, //  30/128
  { -1.52848e-02,  8.90593e-02, -3.20739e-01,  1.27118e+00, -8.05653e-01, -2.89308e-01,  8.49529e-02, -1.48405e-02 }, //  31/128
  { -1.49862e-02,  8.75294e-02, -3.16714e-01,  1.27497e+00, -8.22711e-01, -2.76300e-01,  8.19717e-02, -1.43747e-02 }, //  32/128
  { -1.46789e-02,  8.59479e-02, -3.12502e-01,  1.27843e+00, -8.39517e-01, -2.63347e-01,  7.89727e-02, -1.39039e-02 }, //  33/128
  { -1.43633e-02,  8.43153e-02, -3.08103e-01,  1.28156e+00, -8.56067e-01, -2.50452e-01,  7.59576e-02, -1.34286e-02 }, //  34/128
  { -1.40393e-02,  8.26321e-02, -3.03518e-01,  1.28435e+00, -8.72357e-01, -2.37619e-01,  7.29278e-02, -1.29490e-02 }, //  35/128
  { -1.37071e-02,  8.08990e-02, -2.98747e-01,  1.28682e+00, -8.88384e-01, -2.24851e-01,  6.98849e-02, -1.24653e-02 }, //  36/128
  { -1.33668e-02,  7.91163e-02, -2.93791e-01,  1.28895e+00, -9.04143e-01, -2.12151e-01,  6.68302e-02, -1.19778e-02 }, //  37/128
  { -1.30187e-02,  7.72849e-02, -2.88650e-01,  1.29074e+00, -9.19630e-01, -1.99523e-01,  6.37653e-02, -1.14867e-02 }, //  38/128
  { -1.26627e-02,  7.54052e-02, -2.83325e-01,  1.29220e+00, -9.34844e-01, -1.86970e-01,  6.06916e-02, -1.09923e-02 }, //  39/128
  { -1.22991e-02,  7.34779e-02, -2.77817e-01,  1.29332e+00, -9.49779e-01, -1.74496e-01,  5.76105e-02, -1.04948e-02 }, //  40/128
  { -1.19280e-02,  7.15036e-02, -2.72125e-01,  1.29410e+00, -9.64433e-01, -1.62104e-01,  5.45235e-02, -9.99449e-03 }, //  41/128
  { -1.15496e-02,  6.94831e-02, -2.66252e-01,  1.29453e+00, -9.78802e-01, -1.49797e-01,  5.14320e-02, -9.49163e-03 }, //  42/128
  { -1.11640e-02,  6.74171e-02, -2.60198e-01,  1.29463e+00, -9.92883e-01, -1.37578e-01,  4.83374e-02, -8.98645e-03 }, //  43/128
  { -1.07714e-02,  6.53061e-02, -2.53964e-01,  1.29439e+00, -1.00667e+00, -1.25450e-01,  4.52412e-02, -8.47921e-03 }, //  44/128
  { -1.03719e-02,  6.31511e-02, -2.47551e-01,  1.29380e+00, -1.02017e+00, -1.13418e-01,  4.21447e-02, -7.97016e-03 }, //  45/128
  { -9.96577e-03,  6.09527e-02, -2.40960e-01,  1.29287e+00, -1.03337e+00, -1.01483e-01,  3.90493e-02, -7.45954e-03 }, //  46/128
  { -9.55311e-03,  5.87118e-02, -2.34191e-01,  1.29159e+00, -1.04627e+00, -8.96486e-02,  3.59565e-02, -6.94760e-03 }, //  47/128
  { -9.13410e-03,  5.64291e-02, -2.27247e-01,  1.28997e+00, -1.05886e+00, -7.79181e-02,  3.28675e-02, -6.43460e-03 }, //  48/128
  { -8.70893e-03,  5.41055e-02, -2.20128e-01,  1.28801e+00, -1.07115e+00, -6.62943e-02,  2.97837e-02, -5.92077e-03 }, //  49/128
  { -8.27777e-03,  5.17418e-02, -2.12836e-01,  1.28570e+00, -1.08314e+00, -5.47801e-02,  2.67066e-02, -5.40637e-03 }, //  50/128
  { -7.84082e-03,  4.93389e-02, -2.05372e-01,  1.28305e+00, -1.09481e+00, -4.33784e-02,  2.36373e-02, -4.89163e-03 }, //  51/128
  { -7.39824e-03,  4.68976e-02, -1.97737e-01,  1.28005e+00, -1.10617e+00, -3.20920e-02,  2.05773e-02, -4.37681e-03 }, //  52/128
  { -6.95024e-03,  4.44189e-02, -1.89933e-01,  1.27670e+00, -1.11721e+00, -2.09236e-02,  1.75279e-02, -3.86214e-03 }, //  53/128
  { -6.49701e-03,  4.19036e-02, -1.81961e-01,  1.27302e+00, -1.12794e+00, -9.87606e-03,  1.44903e-02, -3.34787e-03 }, //  54/128
  { -6.03873e-03,  3.93528e-02, -1.73823e-01,  1.26898e+00, -1.13834e+00,  1.04807e-03,  1.14658e-02, -2.83424e-03 }, //  55/128
  { -5.57561e-03,  3.67674e-02, -1.65521e-01,  1.26461e+00, -1.14843e+00,  1.18461e-02,  8.45574e-03, -2.32148e-03 }, //  56/128
  { -5.10786e-03,  3.41483e-02, -1.57056e-01,  1.25989e+00, -1.15819e+00,  2.25155e-02,  5.46133e-03, -1.80983e-03 }, //  57/128
  { -4.63566e-03,  3.14966e-02, -1.48430e-01,  1.25482e+00, -1.16762e+00,  3.30537e-02,  2.48381e-03, -1.29954e-03 }, //  58/128
  { -4.15923e-03,  2.88132e-02, -1.39644e-01,  1.24942e+00, -1.17672e+00,  4.34583e-02, -4.75572e-04, -7.90819e-04 }, //  59/128
  { -3.67878e-03,  2.60992e-02, -1.30701e-01,  1.24367e+00, -1.18550e+00,  5.37267e-02, -3.41562e-03, -2.83916e-04 }, //  60/128
  { -3.19451e-03,  2.33557e-02, -1.21603e-01,  1.23758e+00, -1.19395e+00,  6.38567e-02, -6.33513e-03,  2.20942e-04 }, //  61/128
  { -2.70665e-03,  2.05837e-02, -1.12350e-01,  1.23115e+00, -1.20206e+00,  7.38460e-02, -9.23291e-03,  7.23528e-04 }, //  62/128
  { -2.21539e-03,  1.77843e-02, -1.02946e-01,  1.22439e+00, -1.20984e+00,  8.36921e-02, -1.21078e-02,  1.22361e-03 }, //  63/128
  { -1.72098e-03,  1.49587e-02, -9.33930e-02,  1.21728e+00, -1.21728e+00,  9.33930e-02, -1.49587e-02,  1.72098e-03 }, //  64/128
  { -1.22361e-03,  1.21078e-02, -8.36921e-02,  1.20984e+00, -1.22439e+00,  1.02946e-01, -1.77843e-02,  2.21539e-03 }, //  65/128
  { -7.23528e-04,  9.23291e-03, -7.38460e-02,  1.20206e+00, -1.23115e+00,  1.12350e-01, -2.05837e-02,  2.70665e-03 }, //  66/128
  { -2.20942e-04,  6.33513e-03, -6.38567e-02,  1.19395e+00, -1.23758e+00,  1.21603e-01, -2.33557e-02,  3.19451e-03 }, //  67/128
  {  2.83916e-04,  3.41562e-03, -5.37267e-02,  1.18550e+00, -1.24367e+00,  1.30701e-01, -2.60992e-02,  3.67878e-03 }, //  68/128
  {  7.90819e-04,  4.75572e-04, -4.34583e-02,  1.17672e+00, -1.24942e+00,  1.39644e-01, -2.88132e-02,  4.15923e-03 }, //  69/128
  {  1.29954e-03, -2.48381e-03, -3.30537e-02,  1.16762e+00, -1.25482e+00,  1.48430e-01, -3.14966e-02,  4.63566e-03 }, //  70/128
  {  1.80983e-03, -5.46133e-03, -2.25155e-02,  1.15819e+00, -1.25989e+00,  1.57056e-01, -3.41483e-02,  5.10786e-03 }, //  71/128
  {  2.32148e-03, -8.45574e-03, -1.18461e-02,  1.14843e+00, -1.26461e+00,  1.65521e-01, -3.67674e-02,  5.57561e-03 }, //  72/128
  {  2.83424e-03, -1.14658e-02, -1.04807e-03,  1.13834e+00, -1.26898e+00,  1.73823e-01, -3.93528e-02,  6.03873e-03 }, //  73/128
  {  3.34787e-03, -1.44903e-02,  9.87606e-03,  1.12794e+00, -1.27302e+00,  1.81961e-01, -4.19036e-02,  6.49701e-03 }, //  74/128
  {  3.86214e-03, -1.75279e-02,  2.09236e-02,  1.11721e+00, -1.27670e+00,  1.89933e-01, -4.44189e-02,  6.95024e-03 }, //  75/128
  {  4.37681e-03, -2.05773e-02,  3.20920e-02,  1.10617e+00, -1.28005e+00,  1.97737e-01, -4.68976e-02,  7.39824e-03 }, //  76/128
  {  4.89163e-03, -2.36373e-02,  4.33784e-02,  1.09481e+00, -1.28305e+00,  2.05372e-01, -4.93389e-02,  7.84082e-03 }, //  77/128
  {  5.40637e-03, -2.67066e-02,  5.47801e-02,  1.08314e+00, -1.28570e+00,  2.12836e-01, -5.17418e-02,  8.27777e-03 }, //  78/128
  {  5.92077e-03, -2.97837e-02,  6.62943e-02,  1.07115e+00, -1.28801e+00,  2.20128e-01, -5.41055e-02,  8.70893e-03 }, //  79/128
  {  6.43460e-03, -3.28675e-02,  7.79181e-02,  1.05886e+00, -1.28997e+00,  2.27247e-01, -5.64291e-02,  9.13410e-03 }, //  80/128
  {  6.94760e-03, -3.59565e-02,  8.96486e-02,  1.04627e+00, -1.29159e+00,  2.34191e-01, -5.87118e-02,  9.55311e-03 }, //  81/128
  {  7.45954e-03, -3.90493e-02,  1.01483e-01,  1.03337e+00, -1.29287e+00,  2.40960e-01, -6.09527e-02,  9.96577e-03 }, //  82/128
  {  7.97016e-03, -4.21447e-02,  1.13418e-01,  1.02017e+00, -1.29380e+00,  2.47551e-01, -6.31511e-02,  1.03719e-02 }, //  83/128
  {  8.47921e-03, -4.52412e-02,  1.25450e-01,  1.00667e+00, -1.29439e+00,  2.53964e-01, -6.53061e-02,  1.07714e-02 }, //  84/128
  {  8.98645e-03, -4.83374e-02,  1.37578e-01,  9.92883e-01, -1.29463e+00,  2.60198e-01, -6.74171e-02,  1.11640e-02 }, //  85/128
  {  9.49163e-03, -5.14320e-02,  1.49797e-01,  9.78802e-01, -1.29453e+00,  2.66252e-01, -6.94831e-02,  1.15496e-02 }, //  86/128
  {  9.99449e-03, -5.45235e-02,  1.62104e-01,  9.64433e-01, -1.29410e+00,  2.72125e-01, -7.15036e-02,  1.19280e-02 }, //  87/128
  {  1.04948e-02, -5.76105e-02,  1.74496e-01,  9.49779e-01, -1.29332e+00,  2.77817e-01, -7.34779e-02,  1.22991e-02 }, //  88/128
  {  1.09923e-02, -6.06916e-02,  1.86970e-01,  9.34844e-01, -1.29220e+00,  2.83325e-01, -7.54052e-02,  1.26627e-02 }, //  89/128
  {  1.14867e-02, -6.37653e-02,  1.99523e-01,  9.19630e-01, -1.29074e+00,  2.88650e-01, -7.72849e-02,  1.30187e-02 }, //  90/128
  {  1.19778e-02, -6.68302e-02,  2.12151e-01,  9.04143e-01, -1.28895e+00,  2.93791e-01, -7.91163e-02,  1.33668e-02 }, //  91/128
  {  1.24653e-02, -6.98849e-02,  2.24851e-01,  8.88384e-01, -1.28682e+00,  2.98747e-01, -8.08990e-02,  1.37071e-02 }, //  92/128
  {  1.29490e-02, -7.29278e-02,  2.37619e-01,  8.72357e-01, -1.28435e+00,  3.03518e-01, -8.26321e-02,  1.40393e-02 }, //  93/128
  {  1.34286e-02, -7.59576e-02,  2.50452e-01,  8.56067e-01, -1.28156e+00,  3.08103e-01, -8.43153e-02,  1.43633e-02 }, //  94/128
  {  1.39039e-02, -7.89727e-02,  2.63347e-01,  8.39517e-01, -1.27843e+00,  3.12502e-01, -8.59479e-02,  1.46789e-02 }, //  95/128
  {  1.43747e-02, -8.19717e-02,  2.76300e-01,  8.22711e-01, -1.27497e+00,  3.16714e-01, -8.75294e-02,  1.49862e-02 }, //  96/128
  {  1.48406e-02, -8.49530e-02,  2.89308e-01,  8.05653e-01, -1.27118e+00,  3.20739e-01, -8.90593e-02,  1.52848e-02 }, //  97/128
  {  1.53014e-02, -8.79152e-02,  3.02366e-01,  7.88347e-01, -1.26707e+00,  3.24577e-01, -9.05371e-02,  1.55747e-02 }, //  98/128
  {  1.57569e-02, -9.08568e-02,  3.15472e-01,  7.70796e-01, -1.26263e+00,  3.28227e-01, -9.19624e-02,  1.58558e-02 }, //  99/128
  {  1.62068e-02, -9.37762e-02,  3.28622e-01,  7.53006e-01, -1.25786e+00,  3.31690e-01, -9.33347e-02,  1.61280e-02 }, // 100/128
  {  1.66509e-02, -9.66721e-02,  3.41811e-01,  7.34980e-01, -1.25278e+00,  3.34965e-01, -9.46536e-02,  1.63911e-02 }, // 101/128
  {  1.70889e-02, -9.95428e-02,  3.55037e-01,  7.16722e-01, -1.24738e+00,  3.38052e-01, -9.59188e-02,  1.66451e-02 }, // 102/128
  {  1.75206e-02, -1.02387e-01,  3.68296e-01,  6.98238e-01, -1.24166e+00,  3.40952e-01, -9.71297e-02,  1.68899e-02 }, // 103/128
  {  1.79458e-02, -1.05203e-01,  3.81583e-01,  6.79531e-01, -1.23563e+00,  3.43664e-01, -9.82862e-02,  1.71253e-02 }, // 104/128
  {  1.83641e-02, -1.07989e-01,  3.94895e-01,  6.60606e-01, -1.22928e+00,  3.46188e-01, -9.93878e-02,  1.73513e-02 }, // 105/128
  {  1.87754e-02, -1.10744e-01,  4.08229e-01,  6.41467e-01, -1.22263e+00,  3.48526e-01, -1.00434e-01,  1.75678e-02 }, // 106/128
  {  1.91794e-02, -1.13466e-01,  4.21579e-01,  6.22120e-01, -1.21567e+00,  3.50676e-01, -1.01425e-01,  1.77746e-02 }, // 107/128
  {  1.95759e-02, -1.16154e-01,  4.34944e-01,  6.02568e-01, -1.20840e+00,  3.52639e-01, -1.02361e-01,  1.79718e-02 }, // 108/128
  {  1.99646e-02, -1.18806e-01,  4.48318e-01,  5.82817e-01, -1.20083e+00,  3.54416e-01, -1.03240e-01,  1.81593e-02 }, // 109/128
  {  2.03452e-02, -1.21421e-01,  4.61698e-01,  5.62871e-01, -1.19297e+00,  3.56008e-01, -1.04064e-01,  1.83369e-02 }, // 110/128
  {  2.07177e-02, -1.23997e-01,  4.75079e-01,  5.42735e-01, -1.18481e+00,  3.57414e-01, -1.04831e-01,  1.85046e-02 }, // 111/128
  {  2.10816e-02, -1.26533e-01,  4.88459e-01,  5.22415e-01, -1.17635e+00,  3.58635e-01, -1.05541e-01,  1.86623e-02 }, // 112/128
  {  2.14369e-02, -1.29027e-01,  5.01832e-01,  5.01915e-01, -1.16761e+00,  3.59672e-01, -1.06195e-01,  1.88101e-02 }, // 113/128
  {  2.17832e-02, -1.31477e-01,  5.15195e-01,  4.81241e-01, -1.15857e+00,  3.60525e-01, -1.06793e-01,  1.89477e-02 }, // 114/128
  {  2.21203e-02, -1.33883e-01,  5.28544e-01,  4.60397e-01, -1.14926e+00,  3.61195e-01, -1.07333e-01,  1.90753e-02 }, // 115/128
  {  2.24481e-02, -1.36242e-01,  5.41875e-01,  4.39388e-01, -1.13966e+00,  3.61683e-01, -1.07816e-01,  1.91927e-02 }, // 116/128
  {  2.27662e-02, -1.38553e-01,  5.55183e-01,  4.18221e-01, -1.12979e+00,  3.61990e-01, -1.08243e-01,  1.92998e-02 }, // 117/128
  {  2.30745e-02, -1.40814e-01,  5.68465e-01,  3.96899e-01, -1.11965e+00,  3.62117e-01, -1.08612e-01,  1.93968e-02 }, // 118/128
  {  2.33727e-02, -1.43025e-01,  5.81717e-01,  3.75429e-01, -1.10923e+00,  3.62064e-01, -1.08925e-01,  1.94834e-02 }, // 119/128
  {  2.36607e-02, -1.45183e-01,  5.94935e-01,  3.53816e-01, -1.09855e+00,  3.61833e-01, -1.09180e-01,  1.95598e-02 }, // 120/128
  {  2.39381e-02, -1.47287e-01,  6.08114e-01,  3.32066e-01, -1.08760e+00,  3.61425e-01, -1.09379e-01,  1.96258e-02 }, // 121/128
  {  2.42049e-02, -1.49335e-01,  6.21250e-01,  3.10183e-01, -1.07640e+00,  3.60840e-01, -1.09520e-01,  1.96814e-02 }, // 122/128
  {  2.44607e-02, -1.51327e-01,  6.34339e-01,  2.88174e-01, -1.06494e+00,  3.60080e-01, -1.09605e-01,  1.97267e-02 }, // 123/128
  {  2.47055e-02, -1.53260e-01,  6.47378e-01,  2.66043e-01, -1.05322e+00,  3.59146e-01, -1.09633e-01,  1.97616e-02 }, // 124/128
  {  2.49389e-02, -1.55133e-01,  6.60361e-01,  2.43798e-01, -1.04126e+00,  3.58039e-01, -1.09604e-01,  1.97862e-02 }, // 125/128
  {  2.51608e-02, -1.56945e-01,  6.73286e-01,  2.21442e-01, -1.02906e+00,  3.56761e-01, -1.09519e-01,  1.98003e-02 }, // 126/128
  {  2.53710e-02, -1.58694e-01,  6.86147e-01,  1.98983e-01, -1.01662e+00,  3.55314e-01, -1.09378e-01,  1.98041e-02 }, // 127/128
  {  2.55693e-02, -1.60378e-01,  6.98940e-01,  1.76425e-01, -1.00394e+00,  3.53697e-01, -1.09180e-01,  1.97975e-02 }, // 128/128
};

