/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "nmpc_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int nmpc_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.state[0] = nmpcVariables.x[lRun1 * 9];
nmpcWorkspace.state[1] = nmpcVariables.x[lRun1 * 9 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[lRun1 * 9 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[lRun1 * 9 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[lRun1 * 9 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[lRun1 * 9 + 5];
nmpcWorkspace.state[6] = nmpcVariables.x[lRun1 * 9 + 6];
nmpcWorkspace.state[7] = nmpcVariables.x[lRun1 * 9 + 7];
nmpcWorkspace.state[8] = nmpcVariables.x[lRun1 * 9 + 8];

nmpcWorkspace.state[126] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.state[127] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.state[128] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.state[129] = nmpcVariables.u[lRun1 * 4 + 3];

ret = nmpc_integrate(nmpcWorkspace.state, 1);

nmpcWorkspace.d[lRun1 * 9] = nmpcWorkspace.state[0] - nmpcVariables.x[lRun1 * 9 + 9];
nmpcWorkspace.d[lRun1 * 9 + 1] = nmpcWorkspace.state[1] - nmpcVariables.x[lRun1 * 9 + 10];
nmpcWorkspace.d[lRun1 * 9 + 2] = nmpcWorkspace.state[2] - nmpcVariables.x[lRun1 * 9 + 11];
nmpcWorkspace.d[lRun1 * 9 + 3] = nmpcWorkspace.state[3] - nmpcVariables.x[lRun1 * 9 + 12];
nmpcWorkspace.d[lRun1 * 9 + 4] = nmpcWorkspace.state[4] - nmpcVariables.x[lRun1 * 9 + 13];
nmpcWorkspace.d[lRun1 * 9 + 5] = nmpcWorkspace.state[5] - nmpcVariables.x[lRun1 * 9 + 14];
nmpcWorkspace.d[lRun1 * 9 + 6] = nmpcWorkspace.state[6] - nmpcVariables.x[lRun1 * 9 + 15];
nmpcWorkspace.d[lRun1 * 9 + 7] = nmpcWorkspace.state[7] - nmpcVariables.x[lRun1 * 9 + 16];
nmpcWorkspace.d[lRun1 * 9 + 8] = nmpcWorkspace.state[8] - nmpcVariables.x[lRun1 * 9 + 17];

nmpcWorkspace.evGx[lRun1 * 81] = nmpcWorkspace.state[9];
nmpcWorkspace.evGx[lRun1 * 81 + 1] = nmpcWorkspace.state[10];
nmpcWorkspace.evGx[lRun1 * 81 + 2] = nmpcWorkspace.state[11];
nmpcWorkspace.evGx[lRun1 * 81 + 3] = nmpcWorkspace.state[12];
nmpcWorkspace.evGx[lRun1 * 81 + 4] = nmpcWorkspace.state[13];
nmpcWorkspace.evGx[lRun1 * 81 + 5] = nmpcWorkspace.state[14];
nmpcWorkspace.evGx[lRun1 * 81 + 6] = nmpcWorkspace.state[15];
nmpcWorkspace.evGx[lRun1 * 81 + 7] = nmpcWorkspace.state[16];
nmpcWorkspace.evGx[lRun1 * 81 + 8] = nmpcWorkspace.state[17];
nmpcWorkspace.evGx[lRun1 * 81 + 9] = nmpcWorkspace.state[18];
nmpcWorkspace.evGx[lRun1 * 81 + 10] = nmpcWorkspace.state[19];
nmpcWorkspace.evGx[lRun1 * 81 + 11] = nmpcWorkspace.state[20];
nmpcWorkspace.evGx[lRun1 * 81 + 12] = nmpcWorkspace.state[21];
nmpcWorkspace.evGx[lRun1 * 81 + 13] = nmpcWorkspace.state[22];
nmpcWorkspace.evGx[lRun1 * 81 + 14] = nmpcWorkspace.state[23];
nmpcWorkspace.evGx[lRun1 * 81 + 15] = nmpcWorkspace.state[24];
nmpcWorkspace.evGx[lRun1 * 81 + 16] = nmpcWorkspace.state[25];
nmpcWorkspace.evGx[lRun1 * 81 + 17] = nmpcWorkspace.state[26];
nmpcWorkspace.evGx[lRun1 * 81 + 18] = nmpcWorkspace.state[27];
nmpcWorkspace.evGx[lRun1 * 81 + 19] = nmpcWorkspace.state[28];
nmpcWorkspace.evGx[lRun1 * 81 + 20] = nmpcWorkspace.state[29];
nmpcWorkspace.evGx[lRun1 * 81 + 21] = nmpcWorkspace.state[30];
nmpcWorkspace.evGx[lRun1 * 81 + 22] = nmpcWorkspace.state[31];
nmpcWorkspace.evGx[lRun1 * 81 + 23] = nmpcWorkspace.state[32];
nmpcWorkspace.evGx[lRun1 * 81 + 24] = nmpcWorkspace.state[33];
nmpcWorkspace.evGx[lRun1 * 81 + 25] = nmpcWorkspace.state[34];
nmpcWorkspace.evGx[lRun1 * 81 + 26] = nmpcWorkspace.state[35];
nmpcWorkspace.evGx[lRun1 * 81 + 27] = nmpcWorkspace.state[36];
nmpcWorkspace.evGx[lRun1 * 81 + 28] = nmpcWorkspace.state[37];
nmpcWorkspace.evGx[lRun1 * 81 + 29] = nmpcWorkspace.state[38];
nmpcWorkspace.evGx[lRun1 * 81 + 30] = nmpcWorkspace.state[39];
nmpcWorkspace.evGx[lRun1 * 81 + 31] = nmpcWorkspace.state[40];
nmpcWorkspace.evGx[lRun1 * 81 + 32] = nmpcWorkspace.state[41];
nmpcWorkspace.evGx[lRun1 * 81 + 33] = nmpcWorkspace.state[42];
nmpcWorkspace.evGx[lRun1 * 81 + 34] = nmpcWorkspace.state[43];
nmpcWorkspace.evGx[lRun1 * 81 + 35] = nmpcWorkspace.state[44];
nmpcWorkspace.evGx[lRun1 * 81 + 36] = nmpcWorkspace.state[45];
nmpcWorkspace.evGx[lRun1 * 81 + 37] = nmpcWorkspace.state[46];
nmpcWorkspace.evGx[lRun1 * 81 + 38] = nmpcWorkspace.state[47];
nmpcWorkspace.evGx[lRun1 * 81 + 39] = nmpcWorkspace.state[48];
nmpcWorkspace.evGx[lRun1 * 81 + 40] = nmpcWorkspace.state[49];
nmpcWorkspace.evGx[lRun1 * 81 + 41] = nmpcWorkspace.state[50];
nmpcWorkspace.evGx[lRun1 * 81 + 42] = nmpcWorkspace.state[51];
nmpcWorkspace.evGx[lRun1 * 81 + 43] = nmpcWorkspace.state[52];
nmpcWorkspace.evGx[lRun1 * 81 + 44] = nmpcWorkspace.state[53];
nmpcWorkspace.evGx[lRun1 * 81 + 45] = nmpcWorkspace.state[54];
nmpcWorkspace.evGx[lRun1 * 81 + 46] = nmpcWorkspace.state[55];
nmpcWorkspace.evGx[lRun1 * 81 + 47] = nmpcWorkspace.state[56];
nmpcWorkspace.evGx[lRun1 * 81 + 48] = nmpcWorkspace.state[57];
nmpcWorkspace.evGx[lRun1 * 81 + 49] = nmpcWorkspace.state[58];
nmpcWorkspace.evGx[lRun1 * 81 + 50] = nmpcWorkspace.state[59];
nmpcWorkspace.evGx[lRun1 * 81 + 51] = nmpcWorkspace.state[60];
nmpcWorkspace.evGx[lRun1 * 81 + 52] = nmpcWorkspace.state[61];
nmpcWorkspace.evGx[lRun1 * 81 + 53] = nmpcWorkspace.state[62];
nmpcWorkspace.evGx[lRun1 * 81 + 54] = nmpcWorkspace.state[63];
nmpcWorkspace.evGx[lRun1 * 81 + 55] = nmpcWorkspace.state[64];
nmpcWorkspace.evGx[lRun1 * 81 + 56] = nmpcWorkspace.state[65];
nmpcWorkspace.evGx[lRun1 * 81 + 57] = nmpcWorkspace.state[66];
nmpcWorkspace.evGx[lRun1 * 81 + 58] = nmpcWorkspace.state[67];
nmpcWorkspace.evGx[lRun1 * 81 + 59] = nmpcWorkspace.state[68];
nmpcWorkspace.evGx[lRun1 * 81 + 60] = nmpcWorkspace.state[69];
nmpcWorkspace.evGx[lRun1 * 81 + 61] = nmpcWorkspace.state[70];
nmpcWorkspace.evGx[lRun1 * 81 + 62] = nmpcWorkspace.state[71];
nmpcWorkspace.evGx[lRun1 * 81 + 63] = nmpcWorkspace.state[72];
nmpcWorkspace.evGx[lRun1 * 81 + 64] = nmpcWorkspace.state[73];
nmpcWorkspace.evGx[lRun1 * 81 + 65] = nmpcWorkspace.state[74];
nmpcWorkspace.evGx[lRun1 * 81 + 66] = nmpcWorkspace.state[75];
nmpcWorkspace.evGx[lRun1 * 81 + 67] = nmpcWorkspace.state[76];
nmpcWorkspace.evGx[lRun1 * 81 + 68] = nmpcWorkspace.state[77];
nmpcWorkspace.evGx[lRun1 * 81 + 69] = nmpcWorkspace.state[78];
nmpcWorkspace.evGx[lRun1 * 81 + 70] = nmpcWorkspace.state[79];
nmpcWorkspace.evGx[lRun1 * 81 + 71] = nmpcWorkspace.state[80];
nmpcWorkspace.evGx[lRun1 * 81 + 72] = nmpcWorkspace.state[81];
nmpcWorkspace.evGx[lRun1 * 81 + 73] = nmpcWorkspace.state[82];
nmpcWorkspace.evGx[lRun1 * 81 + 74] = nmpcWorkspace.state[83];
nmpcWorkspace.evGx[lRun1 * 81 + 75] = nmpcWorkspace.state[84];
nmpcWorkspace.evGx[lRun1 * 81 + 76] = nmpcWorkspace.state[85];
nmpcWorkspace.evGx[lRun1 * 81 + 77] = nmpcWorkspace.state[86];
nmpcWorkspace.evGx[lRun1 * 81 + 78] = nmpcWorkspace.state[87];
nmpcWorkspace.evGx[lRun1 * 81 + 79] = nmpcWorkspace.state[88];
nmpcWorkspace.evGx[lRun1 * 81 + 80] = nmpcWorkspace.state[89];

nmpcWorkspace.evGu[lRun1 * 36] = nmpcWorkspace.state[90];
nmpcWorkspace.evGu[lRun1 * 36 + 1] = nmpcWorkspace.state[91];
nmpcWorkspace.evGu[lRun1 * 36 + 2] = nmpcWorkspace.state[92];
nmpcWorkspace.evGu[lRun1 * 36 + 3] = nmpcWorkspace.state[93];
nmpcWorkspace.evGu[lRun1 * 36 + 4] = nmpcWorkspace.state[94];
nmpcWorkspace.evGu[lRun1 * 36 + 5] = nmpcWorkspace.state[95];
nmpcWorkspace.evGu[lRun1 * 36 + 6] = nmpcWorkspace.state[96];
nmpcWorkspace.evGu[lRun1 * 36 + 7] = nmpcWorkspace.state[97];
nmpcWorkspace.evGu[lRun1 * 36 + 8] = nmpcWorkspace.state[98];
nmpcWorkspace.evGu[lRun1 * 36 + 9] = nmpcWorkspace.state[99];
nmpcWorkspace.evGu[lRun1 * 36 + 10] = nmpcWorkspace.state[100];
nmpcWorkspace.evGu[lRun1 * 36 + 11] = nmpcWorkspace.state[101];
nmpcWorkspace.evGu[lRun1 * 36 + 12] = nmpcWorkspace.state[102];
nmpcWorkspace.evGu[lRun1 * 36 + 13] = nmpcWorkspace.state[103];
nmpcWorkspace.evGu[lRun1 * 36 + 14] = nmpcWorkspace.state[104];
nmpcWorkspace.evGu[lRun1 * 36 + 15] = nmpcWorkspace.state[105];
nmpcWorkspace.evGu[lRun1 * 36 + 16] = nmpcWorkspace.state[106];
nmpcWorkspace.evGu[lRun1 * 36 + 17] = nmpcWorkspace.state[107];
nmpcWorkspace.evGu[lRun1 * 36 + 18] = nmpcWorkspace.state[108];
nmpcWorkspace.evGu[lRun1 * 36 + 19] = nmpcWorkspace.state[109];
nmpcWorkspace.evGu[lRun1 * 36 + 20] = nmpcWorkspace.state[110];
nmpcWorkspace.evGu[lRun1 * 36 + 21] = nmpcWorkspace.state[111];
nmpcWorkspace.evGu[lRun1 * 36 + 22] = nmpcWorkspace.state[112];
nmpcWorkspace.evGu[lRun1 * 36 + 23] = nmpcWorkspace.state[113];
nmpcWorkspace.evGu[lRun1 * 36 + 24] = nmpcWorkspace.state[114];
nmpcWorkspace.evGu[lRun1 * 36 + 25] = nmpcWorkspace.state[115];
nmpcWorkspace.evGu[lRun1 * 36 + 26] = nmpcWorkspace.state[116];
nmpcWorkspace.evGu[lRun1 * 36 + 27] = nmpcWorkspace.state[117];
nmpcWorkspace.evGu[lRun1 * 36 + 28] = nmpcWorkspace.state[118];
nmpcWorkspace.evGu[lRun1 * 36 + 29] = nmpcWorkspace.state[119];
nmpcWorkspace.evGu[lRun1 * 36 + 30] = nmpcWorkspace.state[120];
nmpcWorkspace.evGu[lRun1 * 36 + 31] = nmpcWorkspace.state[121];
nmpcWorkspace.evGu[lRun1 * 36 + 32] = nmpcWorkspace.state[122];
nmpcWorkspace.evGu[lRun1 * 36 + 33] = nmpcWorkspace.state[123];
nmpcWorkspace.evGu[lRun1 * 36 + 34] = nmpcWorkspace.state[124];
nmpcWorkspace.evGu[lRun1 * 36 + 35] = nmpcWorkspace.state[125];
}
return ret;
}

void nmpc_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = u[0];
out[10] = u[1];
out[11] = u[2];
out[12] = u[3];
}

void nmpc_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
}

void nmpc_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ2[48] = +tmpObjS[48];
tmpQ2[49] = +tmpObjS[49];
tmpQ2[50] = +tmpObjS[50];
tmpQ2[51] = +tmpObjS[51];
tmpQ2[52] = +tmpObjS[52];
tmpQ2[53] = +tmpObjS[53];
tmpQ2[54] = +tmpObjS[54];
tmpQ2[55] = +tmpObjS[55];
tmpQ2[56] = +tmpObjS[56];
tmpQ2[57] = +tmpObjS[57];
tmpQ2[58] = +tmpObjS[58];
tmpQ2[59] = +tmpObjS[59];
tmpQ2[60] = +tmpObjS[60];
tmpQ2[61] = +tmpObjS[61];
tmpQ2[62] = +tmpObjS[62];
tmpQ2[63] = +tmpObjS[63];
tmpQ2[64] = +tmpObjS[64];
tmpQ2[65] = +tmpObjS[65];
tmpQ2[66] = +tmpObjS[66];
tmpQ2[67] = +tmpObjS[67];
tmpQ2[68] = +tmpObjS[68];
tmpQ2[69] = +tmpObjS[69];
tmpQ2[70] = +tmpObjS[70];
tmpQ2[71] = +tmpObjS[71];
tmpQ2[72] = +tmpObjS[72];
tmpQ2[73] = +tmpObjS[73];
tmpQ2[74] = +tmpObjS[74];
tmpQ2[75] = +tmpObjS[75];
tmpQ2[76] = +tmpObjS[76];
tmpQ2[77] = +tmpObjS[77];
tmpQ2[78] = +tmpObjS[78];
tmpQ2[79] = +tmpObjS[79];
tmpQ2[80] = +tmpObjS[80];
tmpQ2[81] = +tmpObjS[81];
tmpQ2[82] = +tmpObjS[82];
tmpQ2[83] = +tmpObjS[83];
tmpQ2[84] = +tmpObjS[84];
tmpQ2[85] = +tmpObjS[85];
tmpQ2[86] = +tmpObjS[86];
tmpQ2[87] = +tmpObjS[87];
tmpQ2[88] = +tmpObjS[88];
tmpQ2[89] = +tmpObjS[89];
tmpQ2[90] = +tmpObjS[90];
tmpQ2[91] = +tmpObjS[91];
tmpQ2[92] = +tmpObjS[92];
tmpQ2[93] = +tmpObjS[93];
tmpQ2[94] = +tmpObjS[94];
tmpQ2[95] = +tmpObjS[95];
tmpQ2[96] = +tmpObjS[96];
tmpQ2[97] = +tmpObjS[97];
tmpQ2[98] = +tmpObjS[98];
tmpQ2[99] = +tmpObjS[99];
tmpQ2[100] = +tmpObjS[100];
tmpQ2[101] = +tmpObjS[101];
tmpQ2[102] = +tmpObjS[102];
tmpQ2[103] = +tmpObjS[103];
tmpQ2[104] = +tmpObjS[104];
tmpQ2[105] = +tmpObjS[105];
tmpQ2[106] = +tmpObjS[106];
tmpQ2[107] = +tmpObjS[107];
tmpQ2[108] = +tmpObjS[108];
tmpQ2[109] = +tmpObjS[109];
tmpQ2[110] = +tmpObjS[110];
tmpQ2[111] = +tmpObjS[111];
tmpQ2[112] = +tmpObjS[112];
tmpQ2[113] = +tmpObjS[113];
tmpQ2[114] = +tmpObjS[114];
tmpQ2[115] = +tmpObjS[115];
tmpQ2[116] = +tmpObjS[116];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = + tmpQ2[7];
tmpQ1[8] = + tmpQ2[8];
tmpQ1[9] = + tmpQ2[13];
tmpQ1[10] = + tmpQ2[14];
tmpQ1[11] = + tmpQ2[15];
tmpQ1[12] = + tmpQ2[16];
tmpQ1[13] = + tmpQ2[17];
tmpQ1[14] = + tmpQ2[18];
tmpQ1[15] = + tmpQ2[19];
tmpQ1[16] = + tmpQ2[20];
tmpQ1[17] = + tmpQ2[21];
tmpQ1[18] = + tmpQ2[26];
tmpQ1[19] = + tmpQ2[27];
tmpQ1[20] = + tmpQ2[28];
tmpQ1[21] = + tmpQ2[29];
tmpQ1[22] = + tmpQ2[30];
tmpQ1[23] = + tmpQ2[31];
tmpQ1[24] = + tmpQ2[32];
tmpQ1[25] = + tmpQ2[33];
tmpQ1[26] = + tmpQ2[34];
tmpQ1[27] = + tmpQ2[39];
tmpQ1[28] = + tmpQ2[40];
tmpQ1[29] = + tmpQ2[41];
tmpQ1[30] = + tmpQ2[42];
tmpQ1[31] = + tmpQ2[43];
tmpQ1[32] = + tmpQ2[44];
tmpQ1[33] = + tmpQ2[45];
tmpQ1[34] = + tmpQ2[46];
tmpQ1[35] = + tmpQ2[47];
tmpQ1[36] = + tmpQ2[52];
tmpQ1[37] = + tmpQ2[53];
tmpQ1[38] = + tmpQ2[54];
tmpQ1[39] = + tmpQ2[55];
tmpQ1[40] = + tmpQ2[56];
tmpQ1[41] = + tmpQ2[57];
tmpQ1[42] = + tmpQ2[58];
tmpQ1[43] = + tmpQ2[59];
tmpQ1[44] = + tmpQ2[60];
tmpQ1[45] = + tmpQ2[65];
tmpQ1[46] = + tmpQ2[66];
tmpQ1[47] = + tmpQ2[67];
tmpQ1[48] = + tmpQ2[68];
tmpQ1[49] = + tmpQ2[69];
tmpQ1[50] = + tmpQ2[70];
tmpQ1[51] = + tmpQ2[71];
tmpQ1[52] = + tmpQ2[72];
tmpQ1[53] = + tmpQ2[73];
tmpQ1[54] = + tmpQ2[78];
tmpQ1[55] = + tmpQ2[79];
tmpQ1[56] = + tmpQ2[80];
tmpQ1[57] = + tmpQ2[81];
tmpQ1[58] = + tmpQ2[82];
tmpQ1[59] = + tmpQ2[83];
tmpQ1[60] = + tmpQ2[84];
tmpQ1[61] = + tmpQ2[85];
tmpQ1[62] = + tmpQ2[86];
tmpQ1[63] = + tmpQ2[91];
tmpQ1[64] = + tmpQ2[92];
tmpQ1[65] = + tmpQ2[93];
tmpQ1[66] = + tmpQ2[94];
tmpQ1[67] = + tmpQ2[95];
tmpQ1[68] = + tmpQ2[96];
tmpQ1[69] = + tmpQ2[97];
tmpQ1[70] = + tmpQ2[98];
tmpQ1[71] = + tmpQ2[99];
tmpQ1[72] = + tmpQ2[104];
tmpQ1[73] = + tmpQ2[105];
tmpQ1[74] = + tmpQ2[106];
tmpQ1[75] = + tmpQ2[107];
tmpQ1[76] = + tmpQ2[108];
tmpQ1[77] = + tmpQ2[109];
tmpQ1[78] = + tmpQ2[110];
tmpQ1[79] = + tmpQ2[111];
tmpQ1[80] = + tmpQ2[112];
}

void nmpc_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[117];
tmpR2[1] = +tmpObjS[118];
tmpR2[2] = +tmpObjS[119];
tmpR2[3] = +tmpObjS[120];
tmpR2[4] = +tmpObjS[121];
tmpR2[5] = +tmpObjS[122];
tmpR2[6] = +tmpObjS[123];
tmpR2[7] = +tmpObjS[124];
tmpR2[8] = +tmpObjS[125];
tmpR2[9] = +tmpObjS[126];
tmpR2[10] = +tmpObjS[127];
tmpR2[11] = +tmpObjS[128];
tmpR2[12] = +tmpObjS[129];
tmpR2[13] = +tmpObjS[130];
tmpR2[14] = +tmpObjS[131];
tmpR2[15] = +tmpObjS[132];
tmpR2[16] = +tmpObjS[133];
tmpR2[17] = +tmpObjS[134];
tmpR2[18] = +tmpObjS[135];
tmpR2[19] = +tmpObjS[136];
tmpR2[20] = +tmpObjS[137];
tmpR2[21] = +tmpObjS[138];
tmpR2[22] = +tmpObjS[139];
tmpR2[23] = +tmpObjS[140];
tmpR2[24] = +tmpObjS[141];
tmpR2[25] = +tmpObjS[142];
tmpR2[26] = +tmpObjS[143];
tmpR2[27] = +tmpObjS[144];
tmpR2[28] = +tmpObjS[145];
tmpR2[29] = +tmpObjS[146];
tmpR2[30] = +tmpObjS[147];
tmpR2[31] = +tmpObjS[148];
tmpR2[32] = +tmpObjS[149];
tmpR2[33] = +tmpObjS[150];
tmpR2[34] = +tmpObjS[151];
tmpR2[35] = +tmpObjS[152];
tmpR2[36] = +tmpObjS[153];
tmpR2[37] = +tmpObjS[154];
tmpR2[38] = +tmpObjS[155];
tmpR2[39] = +tmpObjS[156];
tmpR2[40] = +tmpObjS[157];
tmpR2[41] = +tmpObjS[158];
tmpR2[42] = +tmpObjS[159];
tmpR2[43] = +tmpObjS[160];
tmpR2[44] = +tmpObjS[161];
tmpR2[45] = +tmpObjS[162];
tmpR2[46] = +tmpObjS[163];
tmpR2[47] = +tmpObjS[164];
tmpR2[48] = +tmpObjS[165];
tmpR2[49] = +tmpObjS[166];
tmpR2[50] = +tmpObjS[167];
tmpR2[51] = +tmpObjS[168];
tmpR1[0] = + tmpR2[9];
tmpR1[1] = + tmpR2[10];
tmpR1[2] = + tmpR2[11];
tmpR1[3] = + tmpR2[12];
tmpR1[4] = + tmpR2[22];
tmpR1[5] = + tmpR2[23];
tmpR1[6] = + tmpR2[24];
tmpR1[7] = + tmpR2[25];
tmpR1[8] = + tmpR2[35];
tmpR1[9] = + tmpR2[36];
tmpR1[10] = + tmpR2[37];
tmpR1[11] = + tmpR2[38];
tmpR1[12] = + tmpR2[48];
tmpR1[13] = + tmpR2[49];
tmpR1[14] = + tmpR2[50];
tmpR1[15] = + tmpR2[51];
}

void nmpc_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = +tmpObjSEndTerm[36];
tmpQN2[37] = +tmpObjSEndTerm[37];
tmpQN2[38] = +tmpObjSEndTerm[38];
tmpQN2[39] = +tmpObjSEndTerm[39];
tmpQN2[40] = +tmpObjSEndTerm[40];
tmpQN2[41] = +tmpObjSEndTerm[41];
tmpQN2[42] = +tmpObjSEndTerm[42];
tmpQN2[43] = +tmpObjSEndTerm[43];
tmpQN2[44] = +tmpObjSEndTerm[44];
tmpQN2[45] = +tmpObjSEndTerm[45];
tmpQN2[46] = +tmpObjSEndTerm[46];
tmpQN2[47] = +tmpObjSEndTerm[47];
tmpQN2[48] = +tmpObjSEndTerm[48];
tmpQN2[49] = +tmpObjSEndTerm[49];
tmpQN2[50] = +tmpObjSEndTerm[50];
tmpQN2[51] = +tmpObjSEndTerm[51];
tmpQN2[52] = +tmpObjSEndTerm[52];
tmpQN2[53] = +tmpObjSEndTerm[53];
tmpQN2[54] = +tmpObjSEndTerm[54];
tmpQN2[55] = +tmpObjSEndTerm[55];
tmpQN2[56] = +tmpObjSEndTerm[56];
tmpQN2[57] = +tmpObjSEndTerm[57];
tmpQN2[58] = +tmpObjSEndTerm[58];
tmpQN2[59] = +tmpObjSEndTerm[59];
tmpQN2[60] = +tmpObjSEndTerm[60];
tmpQN2[61] = +tmpObjSEndTerm[61];
tmpQN2[62] = +tmpObjSEndTerm[62];
tmpQN2[63] = +tmpObjSEndTerm[63];
tmpQN2[64] = +tmpObjSEndTerm[64];
tmpQN2[65] = +tmpObjSEndTerm[65];
tmpQN2[66] = +tmpObjSEndTerm[66];
tmpQN2[67] = +tmpObjSEndTerm[67];
tmpQN2[68] = +tmpObjSEndTerm[68];
tmpQN2[69] = +tmpObjSEndTerm[69];
tmpQN2[70] = +tmpObjSEndTerm[70];
tmpQN2[71] = +tmpObjSEndTerm[71];
tmpQN2[72] = +tmpObjSEndTerm[72];
tmpQN2[73] = +tmpObjSEndTerm[73];
tmpQN2[74] = +tmpObjSEndTerm[74];
tmpQN2[75] = +tmpObjSEndTerm[75];
tmpQN2[76] = +tmpObjSEndTerm[76];
tmpQN2[77] = +tmpObjSEndTerm[77];
tmpQN2[78] = +tmpObjSEndTerm[78];
tmpQN2[79] = +tmpObjSEndTerm[79];
tmpQN2[80] = +tmpObjSEndTerm[80];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
tmpQN1[25] = + tmpQN2[25];
tmpQN1[26] = + tmpQN2[26];
tmpQN1[27] = + tmpQN2[27];
tmpQN1[28] = + tmpQN2[28];
tmpQN1[29] = + tmpQN2[29];
tmpQN1[30] = + tmpQN2[30];
tmpQN1[31] = + tmpQN2[31];
tmpQN1[32] = + tmpQN2[32];
tmpQN1[33] = + tmpQN2[33];
tmpQN1[34] = + tmpQN2[34];
tmpQN1[35] = + tmpQN2[35];
tmpQN1[36] = + tmpQN2[36];
tmpQN1[37] = + tmpQN2[37];
tmpQN1[38] = + tmpQN2[38];
tmpQN1[39] = + tmpQN2[39];
tmpQN1[40] = + tmpQN2[40];
tmpQN1[41] = + tmpQN2[41];
tmpQN1[42] = + tmpQN2[42];
tmpQN1[43] = + tmpQN2[43];
tmpQN1[44] = + tmpQN2[44];
tmpQN1[45] = + tmpQN2[45];
tmpQN1[46] = + tmpQN2[46];
tmpQN1[47] = + tmpQN2[47];
tmpQN1[48] = + tmpQN2[48];
tmpQN1[49] = + tmpQN2[49];
tmpQN1[50] = + tmpQN2[50];
tmpQN1[51] = + tmpQN2[51];
tmpQN1[52] = + tmpQN2[52];
tmpQN1[53] = + tmpQN2[53];
tmpQN1[54] = + tmpQN2[54];
tmpQN1[55] = + tmpQN2[55];
tmpQN1[56] = + tmpQN2[56];
tmpQN1[57] = + tmpQN2[57];
tmpQN1[58] = + tmpQN2[58];
tmpQN1[59] = + tmpQN2[59];
tmpQN1[60] = + tmpQN2[60];
tmpQN1[61] = + tmpQN2[61];
tmpQN1[62] = + tmpQN2[62];
tmpQN1[63] = + tmpQN2[63];
tmpQN1[64] = + tmpQN2[64];
tmpQN1[65] = + tmpQN2[65];
tmpQN1[66] = + tmpQN2[66];
tmpQN1[67] = + tmpQN2[67];
tmpQN1[68] = + tmpQN2[68];
tmpQN1[69] = + tmpQN2[69];
tmpQN1[70] = + tmpQN2[70];
tmpQN1[71] = + tmpQN2[71];
tmpQN1[72] = + tmpQN2[72];
tmpQN1[73] = + tmpQN2[73];
tmpQN1[74] = + tmpQN2[74];
tmpQN1[75] = + tmpQN2[75];
tmpQN1[76] = + tmpQN2[76];
tmpQN1[77] = + tmpQN2[77];
tmpQN1[78] = + tmpQN2[78];
tmpQN1[79] = + tmpQN2[79];
tmpQN1[80] = + tmpQN2[80];
}

void nmpc_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[runObj * 9];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[runObj * 9 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[runObj * 9 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[runObj * 9 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[runObj * 9 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[runObj * 9 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[runObj * 9 + 6];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[runObj * 9 + 7];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[runObj * 9 + 8];
nmpcWorkspace.objValueIn[9] = nmpcVariables.u[runObj * 4];
nmpcWorkspace.objValueIn[10] = nmpcVariables.u[runObj * 4 + 1];
nmpcWorkspace.objValueIn[11] = nmpcVariables.u[runObj * 4 + 2];
nmpcWorkspace.objValueIn[12] = nmpcVariables.u[runObj * 4 + 3];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[runObj * 13] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.Dy[runObj * 13 + 1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.Dy[runObj * 13 + 2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.Dy[runObj * 13 + 3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.Dy[runObj * 13 + 4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.Dy[runObj * 13 + 5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.Dy[runObj * 13 + 6] = nmpcWorkspace.objValueOut[6];
nmpcWorkspace.Dy[runObj * 13 + 7] = nmpcWorkspace.objValueOut[7];
nmpcWorkspace.Dy[runObj * 13 + 8] = nmpcWorkspace.objValueOut[8];
nmpcWorkspace.Dy[runObj * 13 + 9] = nmpcWorkspace.objValueOut[9];
nmpcWorkspace.Dy[runObj * 13 + 10] = nmpcWorkspace.objValueOut[10];
nmpcWorkspace.Dy[runObj * 13 + 11] = nmpcWorkspace.objValueOut[11];
nmpcWorkspace.Dy[runObj * 13 + 12] = nmpcWorkspace.objValueOut[12];

nmpc_setObjQ1Q2( &(nmpcVariables.W[ runObj * 169 ]), &(nmpcWorkspace.Q1[ runObj * 81 ]), &(nmpcWorkspace.Q2[ runObj * 117 ]) );

nmpc_setObjR1R2( &(nmpcVariables.W[ runObj * 169 ]), &(nmpcWorkspace.R1[ runObj * 16 ]), &(nmpcWorkspace.R2[ runObj * 52 ]) );

}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[270];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[271];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[272];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[273];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[274];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[275];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[276];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[277];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[278];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );

nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.DyN[6] = nmpcWorkspace.objValueOut[6];
nmpcWorkspace.DyN[7] = nmpcWorkspace.objValueOut[7];
nmpcWorkspace.DyN[8] = nmpcWorkspace.objValueOut[8];

nmpc_setObjQN1QN2( nmpcVariables.WN, nmpcWorkspace.QN1, nmpcWorkspace.QN2 );

}

void nmpc_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8];
dNew[1] += + Gx1[9]*dOld[0] + Gx1[10]*dOld[1] + Gx1[11]*dOld[2] + Gx1[12]*dOld[3] + Gx1[13]*dOld[4] + Gx1[14]*dOld[5] + Gx1[15]*dOld[6] + Gx1[16]*dOld[7] + Gx1[17]*dOld[8];
dNew[2] += + Gx1[18]*dOld[0] + Gx1[19]*dOld[1] + Gx1[20]*dOld[2] + Gx1[21]*dOld[3] + Gx1[22]*dOld[4] + Gx1[23]*dOld[5] + Gx1[24]*dOld[6] + Gx1[25]*dOld[7] + Gx1[26]*dOld[8];
dNew[3] += + Gx1[27]*dOld[0] + Gx1[28]*dOld[1] + Gx1[29]*dOld[2] + Gx1[30]*dOld[3] + Gx1[31]*dOld[4] + Gx1[32]*dOld[5] + Gx1[33]*dOld[6] + Gx1[34]*dOld[7] + Gx1[35]*dOld[8];
dNew[4] += + Gx1[36]*dOld[0] + Gx1[37]*dOld[1] + Gx1[38]*dOld[2] + Gx1[39]*dOld[3] + Gx1[40]*dOld[4] + Gx1[41]*dOld[5] + Gx1[42]*dOld[6] + Gx1[43]*dOld[7] + Gx1[44]*dOld[8];
dNew[5] += + Gx1[45]*dOld[0] + Gx1[46]*dOld[1] + Gx1[47]*dOld[2] + Gx1[48]*dOld[3] + Gx1[49]*dOld[4] + Gx1[50]*dOld[5] + Gx1[51]*dOld[6] + Gx1[52]*dOld[7] + Gx1[53]*dOld[8];
dNew[6] += + Gx1[54]*dOld[0] + Gx1[55]*dOld[1] + Gx1[56]*dOld[2] + Gx1[57]*dOld[3] + Gx1[58]*dOld[4] + Gx1[59]*dOld[5] + Gx1[60]*dOld[6] + Gx1[61]*dOld[7] + Gx1[62]*dOld[8];
dNew[7] += + Gx1[63]*dOld[0] + Gx1[64]*dOld[1] + Gx1[65]*dOld[2] + Gx1[66]*dOld[3] + Gx1[67]*dOld[4] + Gx1[68]*dOld[5] + Gx1[69]*dOld[6] + Gx1[70]*dOld[7] + Gx1[71]*dOld[8];
dNew[8] += + Gx1[72]*dOld[0] + Gx1[73]*dOld[1] + Gx1[74]*dOld[2] + Gx1[75]*dOld[3] + Gx1[76]*dOld[4] + Gx1[77]*dOld[5] + Gx1[78]*dOld[6] + Gx1[79]*dOld[7] + Gx1[80]*dOld[8];
}

void nmpc_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
Gx2[36] = Gx1[36];
Gx2[37] = Gx1[37];
Gx2[38] = Gx1[38];
Gx2[39] = Gx1[39];
Gx2[40] = Gx1[40];
Gx2[41] = Gx1[41];
Gx2[42] = Gx1[42];
Gx2[43] = Gx1[43];
Gx2[44] = Gx1[44];
Gx2[45] = Gx1[45];
Gx2[46] = Gx1[46];
Gx2[47] = Gx1[47];
Gx2[48] = Gx1[48];
Gx2[49] = Gx1[49];
Gx2[50] = Gx1[50];
Gx2[51] = Gx1[51];
Gx2[52] = Gx1[52];
Gx2[53] = Gx1[53];
Gx2[54] = Gx1[54];
Gx2[55] = Gx1[55];
Gx2[56] = Gx1[56];
Gx2[57] = Gx1[57];
Gx2[58] = Gx1[58];
Gx2[59] = Gx1[59];
Gx2[60] = Gx1[60];
Gx2[61] = Gx1[61];
Gx2[62] = Gx1[62];
Gx2[63] = Gx1[63];
Gx2[64] = Gx1[64];
Gx2[65] = Gx1[65];
Gx2[66] = Gx1[66];
Gx2[67] = Gx1[67];
Gx2[68] = Gx1[68];
Gx2[69] = Gx1[69];
Gx2[70] = Gx1[70];
Gx2[71] = Gx1[71];
Gx2[72] = Gx1[72];
Gx2[73] = Gx1[73];
Gx2[74] = Gx1[74];
Gx2[75] = Gx1[75];
Gx2[76] = Gx1[76];
Gx2[77] = Gx1[77];
Gx2[78] = Gx1[78];
Gx2[79] = Gx1[79];
Gx2[80] = Gx1[80];
}

void nmpc_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[18] + Gx1[3]*Gx2[27] + Gx1[4]*Gx2[36] + Gx1[5]*Gx2[45] + Gx1[6]*Gx2[54] + Gx1[7]*Gx2[63] + Gx1[8]*Gx2[72];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[19] + Gx1[3]*Gx2[28] + Gx1[4]*Gx2[37] + Gx1[5]*Gx2[46] + Gx1[6]*Gx2[55] + Gx1[7]*Gx2[64] + Gx1[8]*Gx2[73];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[20] + Gx1[3]*Gx2[29] + Gx1[4]*Gx2[38] + Gx1[5]*Gx2[47] + Gx1[6]*Gx2[56] + Gx1[7]*Gx2[65] + Gx1[8]*Gx2[74];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[21] + Gx1[3]*Gx2[30] + Gx1[4]*Gx2[39] + Gx1[5]*Gx2[48] + Gx1[6]*Gx2[57] + Gx1[7]*Gx2[66] + Gx1[8]*Gx2[75];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[22] + Gx1[3]*Gx2[31] + Gx1[4]*Gx2[40] + Gx1[5]*Gx2[49] + Gx1[6]*Gx2[58] + Gx1[7]*Gx2[67] + Gx1[8]*Gx2[76];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[23] + Gx1[3]*Gx2[32] + Gx1[4]*Gx2[41] + Gx1[5]*Gx2[50] + Gx1[6]*Gx2[59] + Gx1[7]*Gx2[68] + Gx1[8]*Gx2[77];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[24] + Gx1[3]*Gx2[33] + Gx1[4]*Gx2[42] + Gx1[5]*Gx2[51] + Gx1[6]*Gx2[60] + Gx1[7]*Gx2[69] + Gx1[8]*Gx2[78];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[16] + Gx1[2]*Gx2[25] + Gx1[3]*Gx2[34] + Gx1[4]*Gx2[43] + Gx1[5]*Gx2[52] + Gx1[6]*Gx2[61] + Gx1[7]*Gx2[70] + Gx1[8]*Gx2[79];
Gx3[8] = + Gx1[0]*Gx2[8] + Gx1[1]*Gx2[17] + Gx1[2]*Gx2[26] + Gx1[3]*Gx2[35] + Gx1[4]*Gx2[44] + Gx1[5]*Gx2[53] + Gx1[6]*Gx2[62] + Gx1[7]*Gx2[71] + Gx1[8]*Gx2[80];
Gx3[9] = + Gx1[9]*Gx2[0] + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[18] + Gx1[12]*Gx2[27] + Gx1[13]*Gx2[36] + Gx1[14]*Gx2[45] + Gx1[15]*Gx2[54] + Gx1[16]*Gx2[63] + Gx1[17]*Gx2[72];
Gx3[10] = + Gx1[9]*Gx2[1] + Gx1[10]*Gx2[10] + Gx1[11]*Gx2[19] + Gx1[12]*Gx2[28] + Gx1[13]*Gx2[37] + Gx1[14]*Gx2[46] + Gx1[15]*Gx2[55] + Gx1[16]*Gx2[64] + Gx1[17]*Gx2[73];
Gx3[11] = + Gx1[9]*Gx2[2] + Gx1[10]*Gx2[11] + Gx1[11]*Gx2[20] + Gx1[12]*Gx2[29] + Gx1[13]*Gx2[38] + Gx1[14]*Gx2[47] + Gx1[15]*Gx2[56] + Gx1[16]*Gx2[65] + Gx1[17]*Gx2[74];
Gx3[12] = + Gx1[9]*Gx2[3] + Gx1[10]*Gx2[12] + Gx1[11]*Gx2[21] + Gx1[12]*Gx2[30] + Gx1[13]*Gx2[39] + Gx1[14]*Gx2[48] + Gx1[15]*Gx2[57] + Gx1[16]*Gx2[66] + Gx1[17]*Gx2[75];
Gx3[13] = + Gx1[9]*Gx2[4] + Gx1[10]*Gx2[13] + Gx1[11]*Gx2[22] + Gx1[12]*Gx2[31] + Gx1[13]*Gx2[40] + Gx1[14]*Gx2[49] + Gx1[15]*Gx2[58] + Gx1[16]*Gx2[67] + Gx1[17]*Gx2[76];
Gx3[14] = + Gx1[9]*Gx2[5] + Gx1[10]*Gx2[14] + Gx1[11]*Gx2[23] + Gx1[12]*Gx2[32] + Gx1[13]*Gx2[41] + Gx1[14]*Gx2[50] + Gx1[15]*Gx2[59] + Gx1[16]*Gx2[68] + Gx1[17]*Gx2[77];
Gx3[15] = + Gx1[9]*Gx2[6] + Gx1[10]*Gx2[15] + Gx1[11]*Gx2[24] + Gx1[12]*Gx2[33] + Gx1[13]*Gx2[42] + Gx1[14]*Gx2[51] + Gx1[15]*Gx2[60] + Gx1[16]*Gx2[69] + Gx1[17]*Gx2[78];
Gx3[16] = + Gx1[9]*Gx2[7] + Gx1[10]*Gx2[16] + Gx1[11]*Gx2[25] + Gx1[12]*Gx2[34] + Gx1[13]*Gx2[43] + Gx1[14]*Gx2[52] + Gx1[15]*Gx2[61] + Gx1[16]*Gx2[70] + Gx1[17]*Gx2[79];
Gx3[17] = + Gx1[9]*Gx2[8] + Gx1[10]*Gx2[17] + Gx1[11]*Gx2[26] + Gx1[12]*Gx2[35] + Gx1[13]*Gx2[44] + Gx1[14]*Gx2[53] + Gx1[15]*Gx2[62] + Gx1[16]*Gx2[71] + Gx1[17]*Gx2[80];
Gx3[18] = + Gx1[18]*Gx2[0] + Gx1[19]*Gx2[9] + Gx1[20]*Gx2[18] + Gx1[21]*Gx2[27] + Gx1[22]*Gx2[36] + Gx1[23]*Gx2[45] + Gx1[24]*Gx2[54] + Gx1[25]*Gx2[63] + Gx1[26]*Gx2[72];
Gx3[19] = + Gx1[18]*Gx2[1] + Gx1[19]*Gx2[10] + Gx1[20]*Gx2[19] + Gx1[21]*Gx2[28] + Gx1[22]*Gx2[37] + Gx1[23]*Gx2[46] + Gx1[24]*Gx2[55] + Gx1[25]*Gx2[64] + Gx1[26]*Gx2[73];
Gx3[20] = + Gx1[18]*Gx2[2] + Gx1[19]*Gx2[11] + Gx1[20]*Gx2[20] + Gx1[21]*Gx2[29] + Gx1[22]*Gx2[38] + Gx1[23]*Gx2[47] + Gx1[24]*Gx2[56] + Gx1[25]*Gx2[65] + Gx1[26]*Gx2[74];
Gx3[21] = + Gx1[18]*Gx2[3] + Gx1[19]*Gx2[12] + Gx1[20]*Gx2[21] + Gx1[21]*Gx2[30] + Gx1[22]*Gx2[39] + Gx1[23]*Gx2[48] + Gx1[24]*Gx2[57] + Gx1[25]*Gx2[66] + Gx1[26]*Gx2[75];
Gx3[22] = + Gx1[18]*Gx2[4] + Gx1[19]*Gx2[13] + Gx1[20]*Gx2[22] + Gx1[21]*Gx2[31] + Gx1[22]*Gx2[40] + Gx1[23]*Gx2[49] + Gx1[24]*Gx2[58] + Gx1[25]*Gx2[67] + Gx1[26]*Gx2[76];
Gx3[23] = + Gx1[18]*Gx2[5] + Gx1[19]*Gx2[14] + Gx1[20]*Gx2[23] + Gx1[21]*Gx2[32] + Gx1[22]*Gx2[41] + Gx1[23]*Gx2[50] + Gx1[24]*Gx2[59] + Gx1[25]*Gx2[68] + Gx1[26]*Gx2[77];
Gx3[24] = + Gx1[18]*Gx2[6] + Gx1[19]*Gx2[15] + Gx1[20]*Gx2[24] + Gx1[21]*Gx2[33] + Gx1[22]*Gx2[42] + Gx1[23]*Gx2[51] + Gx1[24]*Gx2[60] + Gx1[25]*Gx2[69] + Gx1[26]*Gx2[78];
Gx3[25] = + Gx1[18]*Gx2[7] + Gx1[19]*Gx2[16] + Gx1[20]*Gx2[25] + Gx1[21]*Gx2[34] + Gx1[22]*Gx2[43] + Gx1[23]*Gx2[52] + Gx1[24]*Gx2[61] + Gx1[25]*Gx2[70] + Gx1[26]*Gx2[79];
Gx3[26] = + Gx1[18]*Gx2[8] + Gx1[19]*Gx2[17] + Gx1[20]*Gx2[26] + Gx1[21]*Gx2[35] + Gx1[22]*Gx2[44] + Gx1[23]*Gx2[53] + Gx1[24]*Gx2[62] + Gx1[25]*Gx2[71] + Gx1[26]*Gx2[80];
Gx3[27] = + Gx1[27]*Gx2[0] + Gx1[28]*Gx2[9] + Gx1[29]*Gx2[18] + Gx1[30]*Gx2[27] + Gx1[31]*Gx2[36] + Gx1[32]*Gx2[45] + Gx1[33]*Gx2[54] + Gx1[34]*Gx2[63] + Gx1[35]*Gx2[72];
Gx3[28] = + Gx1[27]*Gx2[1] + Gx1[28]*Gx2[10] + Gx1[29]*Gx2[19] + Gx1[30]*Gx2[28] + Gx1[31]*Gx2[37] + Gx1[32]*Gx2[46] + Gx1[33]*Gx2[55] + Gx1[34]*Gx2[64] + Gx1[35]*Gx2[73];
Gx3[29] = + Gx1[27]*Gx2[2] + Gx1[28]*Gx2[11] + Gx1[29]*Gx2[20] + Gx1[30]*Gx2[29] + Gx1[31]*Gx2[38] + Gx1[32]*Gx2[47] + Gx1[33]*Gx2[56] + Gx1[34]*Gx2[65] + Gx1[35]*Gx2[74];
Gx3[30] = + Gx1[27]*Gx2[3] + Gx1[28]*Gx2[12] + Gx1[29]*Gx2[21] + Gx1[30]*Gx2[30] + Gx1[31]*Gx2[39] + Gx1[32]*Gx2[48] + Gx1[33]*Gx2[57] + Gx1[34]*Gx2[66] + Gx1[35]*Gx2[75];
Gx3[31] = + Gx1[27]*Gx2[4] + Gx1[28]*Gx2[13] + Gx1[29]*Gx2[22] + Gx1[30]*Gx2[31] + Gx1[31]*Gx2[40] + Gx1[32]*Gx2[49] + Gx1[33]*Gx2[58] + Gx1[34]*Gx2[67] + Gx1[35]*Gx2[76];
Gx3[32] = + Gx1[27]*Gx2[5] + Gx1[28]*Gx2[14] + Gx1[29]*Gx2[23] + Gx1[30]*Gx2[32] + Gx1[31]*Gx2[41] + Gx1[32]*Gx2[50] + Gx1[33]*Gx2[59] + Gx1[34]*Gx2[68] + Gx1[35]*Gx2[77];
Gx3[33] = + Gx1[27]*Gx2[6] + Gx1[28]*Gx2[15] + Gx1[29]*Gx2[24] + Gx1[30]*Gx2[33] + Gx1[31]*Gx2[42] + Gx1[32]*Gx2[51] + Gx1[33]*Gx2[60] + Gx1[34]*Gx2[69] + Gx1[35]*Gx2[78];
Gx3[34] = + Gx1[27]*Gx2[7] + Gx1[28]*Gx2[16] + Gx1[29]*Gx2[25] + Gx1[30]*Gx2[34] + Gx1[31]*Gx2[43] + Gx1[32]*Gx2[52] + Gx1[33]*Gx2[61] + Gx1[34]*Gx2[70] + Gx1[35]*Gx2[79];
Gx3[35] = + Gx1[27]*Gx2[8] + Gx1[28]*Gx2[17] + Gx1[29]*Gx2[26] + Gx1[30]*Gx2[35] + Gx1[31]*Gx2[44] + Gx1[32]*Gx2[53] + Gx1[33]*Gx2[62] + Gx1[34]*Gx2[71] + Gx1[35]*Gx2[80];
Gx3[36] = + Gx1[36]*Gx2[0] + Gx1[37]*Gx2[9] + Gx1[38]*Gx2[18] + Gx1[39]*Gx2[27] + Gx1[40]*Gx2[36] + Gx1[41]*Gx2[45] + Gx1[42]*Gx2[54] + Gx1[43]*Gx2[63] + Gx1[44]*Gx2[72];
Gx3[37] = + Gx1[36]*Gx2[1] + Gx1[37]*Gx2[10] + Gx1[38]*Gx2[19] + Gx1[39]*Gx2[28] + Gx1[40]*Gx2[37] + Gx1[41]*Gx2[46] + Gx1[42]*Gx2[55] + Gx1[43]*Gx2[64] + Gx1[44]*Gx2[73];
Gx3[38] = + Gx1[36]*Gx2[2] + Gx1[37]*Gx2[11] + Gx1[38]*Gx2[20] + Gx1[39]*Gx2[29] + Gx1[40]*Gx2[38] + Gx1[41]*Gx2[47] + Gx1[42]*Gx2[56] + Gx1[43]*Gx2[65] + Gx1[44]*Gx2[74];
Gx3[39] = + Gx1[36]*Gx2[3] + Gx1[37]*Gx2[12] + Gx1[38]*Gx2[21] + Gx1[39]*Gx2[30] + Gx1[40]*Gx2[39] + Gx1[41]*Gx2[48] + Gx1[42]*Gx2[57] + Gx1[43]*Gx2[66] + Gx1[44]*Gx2[75];
Gx3[40] = + Gx1[36]*Gx2[4] + Gx1[37]*Gx2[13] + Gx1[38]*Gx2[22] + Gx1[39]*Gx2[31] + Gx1[40]*Gx2[40] + Gx1[41]*Gx2[49] + Gx1[42]*Gx2[58] + Gx1[43]*Gx2[67] + Gx1[44]*Gx2[76];
Gx3[41] = + Gx1[36]*Gx2[5] + Gx1[37]*Gx2[14] + Gx1[38]*Gx2[23] + Gx1[39]*Gx2[32] + Gx1[40]*Gx2[41] + Gx1[41]*Gx2[50] + Gx1[42]*Gx2[59] + Gx1[43]*Gx2[68] + Gx1[44]*Gx2[77];
Gx3[42] = + Gx1[36]*Gx2[6] + Gx1[37]*Gx2[15] + Gx1[38]*Gx2[24] + Gx1[39]*Gx2[33] + Gx1[40]*Gx2[42] + Gx1[41]*Gx2[51] + Gx1[42]*Gx2[60] + Gx1[43]*Gx2[69] + Gx1[44]*Gx2[78];
Gx3[43] = + Gx1[36]*Gx2[7] + Gx1[37]*Gx2[16] + Gx1[38]*Gx2[25] + Gx1[39]*Gx2[34] + Gx1[40]*Gx2[43] + Gx1[41]*Gx2[52] + Gx1[42]*Gx2[61] + Gx1[43]*Gx2[70] + Gx1[44]*Gx2[79];
Gx3[44] = + Gx1[36]*Gx2[8] + Gx1[37]*Gx2[17] + Gx1[38]*Gx2[26] + Gx1[39]*Gx2[35] + Gx1[40]*Gx2[44] + Gx1[41]*Gx2[53] + Gx1[42]*Gx2[62] + Gx1[43]*Gx2[71] + Gx1[44]*Gx2[80];
Gx3[45] = + Gx1[45]*Gx2[0] + Gx1[46]*Gx2[9] + Gx1[47]*Gx2[18] + Gx1[48]*Gx2[27] + Gx1[49]*Gx2[36] + Gx1[50]*Gx2[45] + Gx1[51]*Gx2[54] + Gx1[52]*Gx2[63] + Gx1[53]*Gx2[72];
Gx3[46] = + Gx1[45]*Gx2[1] + Gx1[46]*Gx2[10] + Gx1[47]*Gx2[19] + Gx1[48]*Gx2[28] + Gx1[49]*Gx2[37] + Gx1[50]*Gx2[46] + Gx1[51]*Gx2[55] + Gx1[52]*Gx2[64] + Gx1[53]*Gx2[73];
Gx3[47] = + Gx1[45]*Gx2[2] + Gx1[46]*Gx2[11] + Gx1[47]*Gx2[20] + Gx1[48]*Gx2[29] + Gx1[49]*Gx2[38] + Gx1[50]*Gx2[47] + Gx1[51]*Gx2[56] + Gx1[52]*Gx2[65] + Gx1[53]*Gx2[74];
Gx3[48] = + Gx1[45]*Gx2[3] + Gx1[46]*Gx2[12] + Gx1[47]*Gx2[21] + Gx1[48]*Gx2[30] + Gx1[49]*Gx2[39] + Gx1[50]*Gx2[48] + Gx1[51]*Gx2[57] + Gx1[52]*Gx2[66] + Gx1[53]*Gx2[75];
Gx3[49] = + Gx1[45]*Gx2[4] + Gx1[46]*Gx2[13] + Gx1[47]*Gx2[22] + Gx1[48]*Gx2[31] + Gx1[49]*Gx2[40] + Gx1[50]*Gx2[49] + Gx1[51]*Gx2[58] + Gx1[52]*Gx2[67] + Gx1[53]*Gx2[76];
Gx3[50] = + Gx1[45]*Gx2[5] + Gx1[46]*Gx2[14] + Gx1[47]*Gx2[23] + Gx1[48]*Gx2[32] + Gx1[49]*Gx2[41] + Gx1[50]*Gx2[50] + Gx1[51]*Gx2[59] + Gx1[52]*Gx2[68] + Gx1[53]*Gx2[77];
Gx3[51] = + Gx1[45]*Gx2[6] + Gx1[46]*Gx2[15] + Gx1[47]*Gx2[24] + Gx1[48]*Gx2[33] + Gx1[49]*Gx2[42] + Gx1[50]*Gx2[51] + Gx1[51]*Gx2[60] + Gx1[52]*Gx2[69] + Gx1[53]*Gx2[78];
Gx3[52] = + Gx1[45]*Gx2[7] + Gx1[46]*Gx2[16] + Gx1[47]*Gx2[25] + Gx1[48]*Gx2[34] + Gx1[49]*Gx2[43] + Gx1[50]*Gx2[52] + Gx1[51]*Gx2[61] + Gx1[52]*Gx2[70] + Gx1[53]*Gx2[79];
Gx3[53] = + Gx1[45]*Gx2[8] + Gx1[46]*Gx2[17] + Gx1[47]*Gx2[26] + Gx1[48]*Gx2[35] + Gx1[49]*Gx2[44] + Gx1[50]*Gx2[53] + Gx1[51]*Gx2[62] + Gx1[52]*Gx2[71] + Gx1[53]*Gx2[80];
Gx3[54] = + Gx1[54]*Gx2[0] + Gx1[55]*Gx2[9] + Gx1[56]*Gx2[18] + Gx1[57]*Gx2[27] + Gx1[58]*Gx2[36] + Gx1[59]*Gx2[45] + Gx1[60]*Gx2[54] + Gx1[61]*Gx2[63] + Gx1[62]*Gx2[72];
Gx3[55] = + Gx1[54]*Gx2[1] + Gx1[55]*Gx2[10] + Gx1[56]*Gx2[19] + Gx1[57]*Gx2[28] + Gx1[58]*Gx2[37] + Gx1[59]*Gx2[46] + Gx1[60]*Gx2[55] + Gx1[61]*Gx2[64] + Gx1[62]*Gx2[73];
Gx3[56] = + Gx1[54]*Gx2[2] + Gx1[55]*Gx2[11] + Gx1[56]*Gx2[20] + Gx1[57]*Gx2[29] + Gx1[58]*Gx2[38] + Gx1[59]*Gx2[47] + Gx1[60]*Gx2[56] + Gx1[61]*Gx2[65] + Gx1[62]*Gx2[74];
Gx3[57] = + Gx1[54]*Gx2[3] + Gx1[55]*Gx2[12] + Gx1[56]*Gx2[21] + Gx1[57]*Gx2[30] + Gx1[58]*Gx2[39] + Gx1[59]*Gx2[48] + Gx1[60]*Gx2[57] + Gx1[61]*Gx2[66] + Gx1[62]*Gx2[75];
Gx3[58] = + Gx1[54]*Gx2[4] + Gx1[55]*Gx2[13] + Gx1[56]*Gx2[22] + Gx1[57]*Gx2[31] + Gx1[58]*Gx2[40] + Gx1[59]*Gx2[49] + Gx1[60]*Gx2[58] + Gx1[61]*Gx2[67] + Gx1[62]*Gx2[76];
Gx3[59] = + Gx1[54]*Gx2[5] + Gx1[55]*Gx2[14] + Gx1[56]*Gx2[23] + Gx1[57]*Gx2[32] + Gx1[58]*Gx2[41] + Gx1[59]*Gx2[50] + Gx1[60]*Gx2[59] + Gx1[61]*Gx2[68] + Gx1[62]*Gx2[77];
Gx3[60] = + Gx1[54]*Gx2[6] + Gx1[55]*Gx2[15] + Gx1[56]*Gx2[24] + Gx1[57]*Gx2[33] + Gx1[58]*Gx2[42] + Gx1[59]*Gx2[51] + Gx1[60]*Gx2[60] + Gx1[61]*Gx2[69] + Gx1[62]*Gx2[78];
Gx3[61] = + Gx1[54]*Gx2[7] + Gx1[55]*Gx2[16] + Gx1[56]*Gx2[25] + Gx1[57]*Gx2[34] + Gx1[58]*Gx2[43] + Gx1[59]*Gx2[52] + Gx1[60]*Gx2[61] + Gx1[61]*Gx2[70] + Gx1[62]*Gx2[79];
Gx3[62] = + Gx1[54]*Gx2[8] + Gx1[55]*Gx2[17] + Gx1[56]*Gx2[26] + Gx1[57]*Gx2[35] + Gx1[58]*Gx2[44] + Gx1[59]*Gx2[53] + Gx1[60]*Gx2[62] + Gx1[61]*Gx2[71] + Gx1[62]*Gx2[80];
Gx3[63] = + Gx1[63]*Gx2[0] + Gx1[64]*Gx2[9] + Gx1[65]*Gx2[18] + Gx1[66]*Gx2[27] + Gx1[67]*Gx2[36] + Gx1[68]*Gx2[45] + Gx1[69]*Gx2[54] + Gx1[70]*Gx2[63] + Gx1[71]*Gx2[72];
Gx3[64] = + Gx1[63]*Gx2[1] + Gx1[64]*Gx2[10] + Gx1[65]*Gx2[19] + Gx1[66]*Gx2[28] + Gx1[67]*Gx2[37] + Gx1[68]*Gx2[46] + Gx1[69]*Gx2[55] + Gx1[70]*Gx2[64] + Gx1[71]*Gx2[73];
Gx3[65] = + Gx1[63]*Gx2[2] + Gx1[64]*Gx2[11] + Gx1[65]*Gx2[20] + Gx1[66]*Gx2[29] + Gx1[67]*Gx2[38] + Gx1[68]*Gx2[47] + Gx1[69]*Gx2[56] + Gx1[70]*Gx2[65] + Gx1[71]*Gx2[74];
Gx3[66] = + Gx1[63]*Gx2[3] + Gx1[64]*Gx2[12] + Gx1[65]*Gx2[21] + Gx1[66]*Gx2[30] + Gx1[67]*Gx2[39] + Gx1[68]*Gx2[48] + Gx1[69]*Gx2[57] + Gx1[70]*Gx2[66] + Gx1[71]*Gx2[75];
Gx3[67] = + Gx1[63]*Gx2[4] + Gx1[64]*Gx2[13] + Gx1[65]*Gx2[22] + Gx1[66]*Gx2[31] + Gx1[67]*Gx2[40] + Gx1[68]*Gx2[49] + Gx1[69]*Gx2[58] + Gx1[70]*Gx2[67] + Gx1[71]*Gx2[76];
Gx3[68] = + Gx1[63]*Gx2[5] + Gx1[64]*Gx2[14] + Gx1[65]*Gx2[23] + Gx1[66]*Gx2[32] + Gx1[67]*Gx2[41] + Gx1[68]*Gx2[50] + Gx1[69]*Gx2[59] + Gx1[70]*Gx2[68] + Gx1[71]*Gx2[77];
Gx3[69] = + Gx1[63]*Gx2[6] + Gx1[64]*Gx2[15] + Gx1[65]*Gx2[24] + Gx1[66]*Gx2[33] + Gx1[67]*Gx2[42] + Gx1[68]*Gx2[51] + Gx1[69]*Gx2[60] + Gx1[70]*Gx2[69] + Gx1[71]*Gx2[78];
Gx3[70] = + Gx1[63]*Gx2[7] + Gx1[64]*Gx2[16] + Gx1[65]*Gx2[25] + Gx1[66]*Gx2[34] + Gx1[67]*Gx2[43] + Gx1[68]*Gx2[52] + Gx1[69]*Gx2[61] + Gx1[70]*Gx2[70] + Gx1[71]*Gx2[79];
Gx3[71] = + Gx1[63]*Gx2[8] + Gx1[64]*Gx2[17] + Gx1[65]*Gx2[26] + Gx1[66]*Gx2[35] + Gx1[67]*Gx2[44] + Gx1[68]*Gx2[53] + Gx1[69]*Gx2[62] + Gx1[70]*Gx2[71] + Gx1[71]*Gx2[80];
Gx3[72] = + Gx1[72]*Gx2[0] + Gx1[73]*Gx2[9] + Gx1[74]*Gx2[18] + Gx1[75]*Gx2[27] + Gx1[76]*Gx2[36] + Gx1[77]*Gx2[45] + Gx1[78]*Gx2[54] + Gx1[79]*Gx2[63] + Gx1[80]*Gx2[72];
Gx3[73] = + Gx1[72]*Gx2[1] + Gx1[73]*Gx2[10] + Gx1[74]*Gx2[19] + Gx1[75]*Gx2[28] + Gx1[76]*Gx2[37] + Gx1[77]*Gx2[46] + Gx1[78]*Gx2[55] + Gx1[79]*Gx2[64] + Gx1[80]*Gx2[73];
Gx3[74] = + Gx1[72]*Gx2[2] + Gx1[73]*Gx2[11] + Gx1[74]*Gx2[20] + Gx1[75]*Gx2[29] + Gx1[76]*Gx2[38] + Gx1[77]*Gx2[47] + Gx1[78]*Gx2[56] + Gx1[79]*Gx2[65] + Gx1[80]*Gx2[74];
Gx3[75] = + Gx1[72]*Gx2[3] + Gx1[73]*Gx2[12] + Gx1[74]*Gx2[21] + Gx1[75]*Gx2[30] + Gx1[76]*Gx2[39] + Gx1[77]*Gx2[48] + Gx1[78]*Gx2[57] + Gx1[79]*Gx2[66] + Gx1[80]*Gx2[75];
Gx3[76] = + Gx1[72]*Gx2[4] + Gx1[73]*Gx2[13] + Gx1[74]*Gx2[22] + Gx1[75]*Gx2[31] + Gx1[76]*Gx2[40] + Gx1[77]*Gx2[49] + Gx1[78]*Gx2[58] + Gx1[79]*Gx2[67] + Gx1[80]*Gx2[76];
Gx3[77] = + Gx1[72]*Gx2[5] + Gx1[73]*Gx2[14] + Gx1[74]*Gx2[23] + Gx1[75]*Gx2[32] + Gx1[76]*Gx2[41] + Gx1[77]*Gx2[50] + Gx1[78]*Gx2[59] + Gx1[79]*Gx2[68] + Gx1[80]*Gx2[77];
Gx3[78] = + Gx1[72]*Gx2[6] + Gx1[73]*Gx2[15] + Gx1[74]*Gx2[24] + Gx1[75]*Gx2[33] + Gx1[76]*Gx2[42] + Gx1[77]*Gx2[51] + Gx1[78]*Gx2[60] + Gx1[79]*Gx2[69] + Gx1[80]*Gx2[78];
Gx3[79] = + Gx1[72]*Gx2[7] + Gx1[73]*Gx2[16] + Gx1[74]*Gx2[25] + Gx1[75]*Gx2[34] + Gx1[76]*Gx2[43] + Gx1[77]*Gx2[52] + Gx1[78]*Gx2[61] + Gx1[79]*Gx2[70] + Gx1[80]*Gx2[79];
Gx3[80] = + Gx1[72]*Gx2[8] + Gx1[73]*Gx2[17] + Gx1[74]*Gx2[26] + Gx1[75]*Gx2[35] + Gx1[76]*Gx2[44] + Gx1[77]*Gx2[53] + Gx1[78]*Gx2[62] + Gx1[79]*Gx2[71] + Gx1[80]*Gx2[80];
}

void nmpc_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28] + Gx1[8]*Gu1[32];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29] + Gx1[8]*Gu1[33];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30] + Gx1[8]*Gu1[34];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31] + Gx1[8]*Gu1[35];
Gu2[4] = + Gx1[9]*Gu1[0] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[8] + Gx1[12]*Gu1[12] + Gx1[13]*Gu1[16] + Gx1[14]*Gu1[20] + Gx1[15]*Gu1[24] + Gx1[16]*Gu1[28] + Gx1[17]*Gu1[32];
Gu2[5] = + Gx1[9]*Gu1[1] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[9] + Gx1[12]*Gu1[13] + Gx1[13]*Gu1[17] + Gx1[14]*Gu1[21] + Gx1[15]*Gu1[25] + Gx1[16]*Gu1[29] + Gx1[17]*Gu1[33];
Gu2[6] = + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[6] + Gx1[11]*Gu1[10] + Gx1[12]*Gu1[14] + Gx1[13]*Gu1[18] + Gx1[14]*Gu1[22] + Gx1[15]*Gu1[26] + Gx1[16]*Gu1[30] + Gx1[17]*Gu1[34];
Gu2[7] = + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[7] + Gx1[11]*Gu1[11] + Gx1[12]*Gu1[15] + Gx1[13]*Gu1[19] + Gx1[14]*Gu1[23] + Gx1[15]*Gu1[27] + Gx1[16]*Gu1[31] + Gx1[17]*Gu1[35];
Gu2[8] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[4] + Gx1[20]*Gu1[8] + Gx1[21]*Gu1[12] + Gx1[22]*Gu1[16] + Gx1[23]*Gu1[20] + Gx1[24]*Gu1[24] + Gx1[25]*Gu1[28] + Gx1[26]*Gu1[32];
Gu2[9] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[5] + Gx1[20]*Gu1[9] + Gx1[21]*Gu1[13] + Gx1[22]*Gu1[17] + Gx1[23]*Gu1[21] + Gx1[24]*Gu1[25] + Gx1[25]*Gu1[29] + Gx1[26]*Gu1[33];
Gu2[10] = + Gx1[18]*Gu1[2] + Gx1[19]*Gu1[6] + Gx1[20]*Gu1[10] + Gx1[21]*Gu1[14] + Gx1[22]*Gu1[18] + Gx1[23]*Gu1[22] + Gx1[24]*Gu1[26] + Gx1[25]*Gu1[30] + Gx1[26]*Gu1[34];
Gu2[11] = + Gx1[18]*Gu1[3] + Gx1[19]*Gu1[7] + Gx1[20]*Gu1[11] + Gx1[21]*Gu1[15] + Gx1[22]*Gu1[19] + Gx1[23]*Gu1[23] + Gx1[24]*Gu1[27] + Gx1[25]*Gu1[31] + Gx1[26]*Gu1[35];
Gu2[12] = + Gx1[27]*Gu1[0] + Gx1[28]*Gu1[4] + Gx1[29]*Gu1[8] + Gx1[30]*Gu1[12] + Gx1[31]*Gu1[16] + Gx1[32]*Gu1[20] + Gx1[33]*Gu1[24] + Gx1[34]*Gu1[28] + Gx1[35]*Gu1[32];
Gu2[13] = + Gx1[27]*Gu1[1] + Gx1[28]*Gu1[5] + Gx1[29]*Gu1[9] + Gx1[30]*Gu1[13] + Gx1[31]*Gu1[17] + Gx1[32]*Gu1[21] + Gx1[33]*Gu1[25] + Gx1[34]*Gu1[29] + Gx1[35]*Gu1[33];
Gu2[14] = + Gx1[27]*Gu1[2] + Gx1[28]*Gu1[6] + Gx1[29]*Gu1[10] + Gx1[30]*Gu1[14] + Gx1[31]*Gu1[18] + Gx1[32]*Gu1[22] + Gx1[33]*Gu1[26] + Gx1[34]*Gu1[30] + Gx1[35]*Gu1[34];
Gu2[15] = + Gx1[27]*Gu1[3] + Gx1[28]*Gu1[7] + Gx1[29]*Gu1[11] + Gx1[30]*Gu1[15] + Gx1[31]*Gu1[19] + Gx1[32]*Gu1[23] + Gx1[33]*Gu1[27] + Gx1[34]*Gu1[31] + Gx1[35]*Gu1[35];
Gu2[16] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[12] + Gx1[40]*Gu1[16] + Gx1[41]*Gu1[20] + Gx1[42]*Gu1[24] + Gx1[43]*Gu1[28] + Gx1[44]*Gu1[32];
Gu2[17] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[9] + Gx1[39]*Gu1[13] + Gx1[40]*Gu1[17] + Gx1[41]*Gu1[21] + Gx1[42]*Gu1[25] + Gx1[43]*Gu1[29] + Gx1[44]*Gu1[33];
Gu2[18] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[6] + Gx1[38]*Gu1[10] + Gx1[39]*Gu1[14] + Gx1[40]*Gu1[18] + Gx1[41]*Gu1[22] + Gx1[42]*Gu1[26] + Gx1[43]*Gu1[30] + Gx1[44]*Gu1[34];
Gu2[19] = + Gx1[36]*Gu1[3] + Gx1[37]*Gu1[7] + Gx1[38]*Gu1[11] + Gx1[39]*Gu1[15] + Gx1[40]*Gu1[19] + Gx1[41]*Gu1[23] + Gx1[42]*Gu1[27] + Gx1[43]*Gu1[31] + Gx1[44]*Gu1[35];
Gu2[20] = + Gx1[45]*Gu1[0] + Gx1[46]*Gu1[4] + Gx1[47]*Gu1[8] + Gx1[48]*Gu1[12] + Gx1[49]*Gu1[16] + Gx1[50]*Gu1[20] + Gx1[51]*Gu1[24] + Gx1[52]*Gu1[28] + Gx1[53]*Gu1[32];
Gu2[21] = + Gx1[45]*Gu1[1] + Gx1[46]*Gu1[5] + Gx1[47]*Gu1[9] + Gx1[48]*Gu1[13] + Gx1[49]*Gu1[17] + Gx1[50]*Gu1[21] + Gx1[51]*Gu1[25] + Gx1[52]*Gu1[29] + Gx1[53]*Gu1[33];
Gu2[22] = + Gx1[45]*Gu1[2] + Gx1[46]*Gu1[6] + Gx1[47]*Gu1[10] + Gx1[48]*Gu1[14] + Gx1[49]*Gu1[18] + Gx1[50]*Gu1[22] + Gx1[51]*Gu1[26] + Gx1[52]*Gu1[30] + Gx1[53]*Gu1[34];
Gu2[23] = + Gx1[45]*Gu1[3] + Gx1[46]*Gu1[7] + Gx1[47]*Gu1[11] + Gx1[48]*Gu1[15] + Gx1[49]*Gu1[19] + Gx1[50]*Gu1[23] + Gx1[51]*Gu1[27] + Gx1[52]*Gu1[31] + Gx1[53]*Gu1[35];
Gu2[24] = + Gx1[54]*Gu1[0] + Gx1[55]*Gu1[4] + Gx1[56]*Gu1[8] + Gx1[57]*Gu1[12] + Gx1[58]*Gu1[16] + Gx1[59]*Gu1[20] + Gx1[60]*Gu1[24] + Gx1[61]*Gu1[28] + Gx1[62]*Gu1[32];
Gu2[25] = + Gx1[54]*Gu1[1] + Gx1[55]*Gu1[5] + Gx1[56]*Gu1[9] + Gx1[57]*Gu1[13] + Gx1[58]*Gu1[17] + Gx1[59]*Gu1[21] + Gx1[60]*Gu1[25] + Gx1[61]*Gu1[29] + Gx1[62]*Gu1[33];
Gu2[26] = + Gx1[54]*Gu1[2] + Gx1[55]*Gu1[6] + Gx1[56]*Gu1[10] + Gx1[57]*Gu1[14] + Gx1[58]*Gu1[18] + Gx1[59]*Gu1[22] + Gx1[60]*Gu1[26] + Gx1[61]*Gu1[30] + Gx1[62]*Gu1[34];
Gu2[27] = + Gx1[54]*Gu1[3] + Gx1[55]*Gu1[7] + Gx1[56]*Gu1[11] + Gx1[57]*Gu1[15] + Gx1[58]*Gu1[19] + Gx1[59]*Gu1[23] + Gx1[60]*Gu1[27] + Gx1[61]*Gu1[31] + Gx1[62]*Gu1[35];
Gu2[28] = + Gx1[63]*Gu1[0] + Gx1[64]*Gu1[4] + Gx1[65]*Gu1[8] + Gx1[66]*Gu1[12] + Gx1[67]*Gu1[16] + Gx1[68]*Gu1[20] + Gx1[69]*Gu1[24] + Gx1[70]*Gu1[28] + Gx1[71]*Gu1[32];
Gu2[29] = + Gx1[63]*Gu1[1] + Gx1[64]*Gu1[5] + Gx1[65]*Gu1[9] + Gx1[66]*Gu1[13] + Gx1[67]*Gu1[17] + Gx1[68]*Gu1[21] + Gx1[69]*Gu1[25] + Gx1[70]*Gu1[29] + Gx1[71]*Gu1[33];
Gu2[30] = + Gx1[63]*Gu1[2] + Gx1[64]*Gu1[6] + Gx1[65]*Gu1[10] + Gx1[66]*Gu1[14] + Gx1[67]*Gu1[18] + Gx1[68]*Gu1[22] + Gx1[69]*Gu1[26] + Gx1[70]*Gu1[30] + Gx1[71]*Gu1[34];
Gu2[31] = + Gx1[63]*Gu1[3] + Gx1[64]*Gu1[7] + Gx1[65]*Gu1[11] + Gx1[66]*Gu1[15] + Gx1[67]*Gu1[19] + Gx1[68]*Gu1[23] + Gx1[69]*Gu1[27] + Gx1[70]*Gu1[31] + Gx1[71]*Gu1[35];
Gu2[32] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[12] + Gx1[76]*Gu1[16] + Gx1[77]*Gu1[20] + Gx1[78]*Gu1[24] + Gx1[79]*Gu1[28] + Gx1[80]*Gu1[32];
Gu2[33] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[9] + Gx1[75]*Gu1[13] + Gx1[76]*Gu1[17] + Gx1[77]*Gu1[21] + Gx1[78]*Gu1[25] + Gx1[79]*Gu1[29] + Gx1[80]*Gu1[33];
Gu2[34] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[6] + Gx1[74]*Gu1[10] + Gx1[75]*Gu1[14] + Gx1[76]*Gu1[18] + Gx1[77]*Gu1[22] + Gx1[78]*Gu1[26] + Gx1[79]*Gu1[30] + Gx1[80]*Gu1[34];
Gu2[35] = + Gx1[72]*Gu1[3] + Gx1[73]*Gu1[7] + Gx1[74]*Gu1[11] + Gx1[75]*Gu1[15] + Gx1[76]*Gu1[19] + Gx1[77]*Gu1[23] + Gx1[78]*Gu1[27] + Gx1[79]*Gu1[31] + Gx1[80]*Gu1[35];
}

void nmpc_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
}

void nmpc_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
nmpcWorkspace.H[(iRow * 480) + (iCol * 4)] += + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32];
nmpcWorkspace.H[(iRow * 480) + (iCol * 4 + 1)] += + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33];
nmpcWorkspace.H[(iRow * 480) + (iCol * 4 + 2)] += + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34];
nmpcWorkspace.H[(iRow * 480) + (iCol * 4 + 3)] += + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35];
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4)] += + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32];
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 1)] += + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33];
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 2)] += + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34];
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 3)] += + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35];
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4)] += + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32];
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 1)] += + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33];
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 2)] += + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34];
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 3)] += + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35];
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4)] += + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32];
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 1)] += + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33];
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 2)] += + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34];
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 3)] += + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35];
}

void nmpc_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
nmpcWorkspace.H[(iRow * 480) + (iCol * 4)] = R11[0];
nmpcWorkspace.H[(iRow * 480) + (iCol * 4 + 1)] = R11[1];
nmpcWorkspace.H[(iRow * 480) + (iCol * 4 + 2)] = R11[2];
nmpcWorkspace.H[(iRow * 480) + (iCol * 4 + 3)] = R11[3];
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4)] = R11[4];
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 1)] = R11[5];
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 2)] = R11[6];
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 3)] = R11[7];
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4)] = R11[8];
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 1)] = R11[9];
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 2)] = R11[10];
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 3)] = R11[11];
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4)] = R11[12];
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 1)] = R11[13];
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 2)] = R11[14];
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 3)] = R11[15];
}

void nmpc_zeroBlockH11( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 480) + (iCol * 4)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
}

void nmpc_copyHTH( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 480) + (iCol * 4)] = nmpcWorkspace.H[(iCol * 480) + (iRow * 4)];
nmpcWorkspace.H[(iRow * 480) + (iCol * 4 + 1)] = nmpcWorkspace.H[(iCol * 480 + 120) + (iRow * 4)];
nmpcWorkspace.H[(iRow * 480) + (iCol * 4 + 2)] = nmpcWorkspace.H[(iCol * 480 + 240) + (iRow * 4)];
nmpcWorkspace.H[(iRow * 480) + (iCol * 4 + 3)] = nmpcWorkspace.H[(iCol * 480 + 360) + (iRow * 4)];
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4)] = nmpcWorkspace.H[(iCol * 480) + (iRow * 4 + 1)];
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 1)] = nmpcWorkspace.H[(iCol * 480 + 120) + (iRow * 4 + 1)];
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 2)] = nmpcWorkspace.H[(iCol * 480 + 240) + (iRow * 4 + 1)];
nmpcWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 3)] = nmpcWorkspace.H[(iCol * 480 + 360) + (iRow * 4 + 1)];
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4)] = nmpcWorkspace.H[(iCol * 480) + (iRow * 4 + 2)];
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 1)] = nmpcWorkspace.H[(iCol * 480 + 120) + (iRow * 4 + 2)];
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 2)] = nmpcWorkspace.H[(iCol * 480 + 240) + (iRow * 4 + 2)];
nmpcWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 3)] = nmpcWorkspace.H[(iCol * 480 + 360) + (iRow * 4 + 2)];
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4)] = nmpcWorkspace.H[(iCol * 480) + (iRow * 4 + 3)];
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 1)] = nmpcWorkspace.H[(iCol * 480 + 120) + (iRow * 4 + 3)];
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 2)] = nmpcWorkspace.H[(iCol * 480 + 240) + (iRow * 4 + 3)];
nmpcWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 3)] = nmpcWorkspace.H[(iCol * 480 + 360) + (iRow * 4 + 3)];
}

void nmpc_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8];
dNew[1] = + Gx1[9]*dOld[0] + Gx1[10]*dOld[1] + Gx1[11]*dOld[2] + Gx1[12]*dOld[3] + Gx1[13]*dOld[4] + Gx1[14]*dOld[5] + Gx1[15]*dOld[6] + Gx1[16]*dOld[7] + Gx1[17]*dOld[8];
dNew[2] = + Gx1[18]*dOld[0] + Gx1[19]*dOld[1] + Gx1[20]*dOld[2] + Gx1[21]*dOld[3] + Gx1[22]*dOld[4] + Gx1[23]*dOld[5] + Gx1[24]*dOld[6] + Gx1[25]*dOld[7] + Gx1[26]*dOld[8];
dNew[3] = + Gx1[27]*dOld[0] + Gx1[28]*dOld[1] + Gx1[29]*dOld[2] + Gx1[30]*dOld[3] + Gx1[31]*dOld[4] + Gx1[32]*dOld[5] + Gx1[33]*dOld[6] + Gx1[34]*dOld[7] + Gx1[35]*dOld[8];
dNew[4] = + Gx1[36]*dOld[0] + Gx1[37]*dOld[1] + Gx1[38]*dOld[2] + Gx1[39]*dOld[3] + Gx1[40]*dOld[4] + Gx1[41]*dOld[5] + Gx1[42]*dOld[6] + Gx1[43]*dOld[7] + Gx1[44]*dOld[8];
dNew[5] = + Gx1[45]*dOld[0] + Gx1[46]*dOld[1] + Gx1[47]*dOld[2] + Gx1[48]*dOld[3] + Gx1[49]*dOld[4] + Gx1[50]*dOld[5] + Gx1[51]*dOld[6] + Gx1[52]*dOld[7] + Gx1[53]*dOld[8];
dNew[6] = + Gx1[54]*dOld[0] + Gx1[55]*dOld[1] + Gx1[56]*dOld[2] + Gx1[57]*dOld[3] + Gx1[58]*dOld[4] + Gx1[59]*dOld[5] + Gx1[60]*dOld[6] + Gx1[61]*dOld[7] + Gx1[62]*dOld[8];
dNew[7] = + Gx1[63]*dOld[0] + Gx1[64]*dOld[1] + Gx1[65]*dOld[2] + Gx1[66]*dOld[3] + Gx1[67]*dOld[4] + Gx1[68]*dOld[5] + Gx1[69]*dOld[6] + Gx1[70]*dOld[7] + Gx1[71]*dOld[8];
dNew[8] = + Gx1[72]*dOld[0] + Gx1[73]*dOld[1] + Gx1[74]*dOld[2] + Gx1[75]*dOld[3] + Gx1[76]*dOld[4] + Gx1[77]*dOld[5] + Gx1[78]*dOld[6] + Gx1[79]*dOld[7] + Gx1[80]*dOld[8];
}

void nmpc_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + nmpcWorkspace.QN1[0]*dOld[0] + nmpcWorkspace.QN1[1]*dOld[1] + nmpcWorkspace.QN1[2]*dOld[2] + nmpcWorkspace.QN1[3]*dOld[3] + nmpcWorkspace.QN1[4]*dOld[4] + nmpcWorkspace.QN1[5]*dOld[5] + nmpcWorkspace.QN1[6]*dOld[6] + nmpcWorkspace.QN1[7]*dOld[7] + nmpcWorkspace.QN1[8]*dOld[8];
dNew[1] = + nmpcWorkspace.QN1[9]*dOld[0] + nmpcWorkspace.QN1[10]*dOld[1] + nmpcWorkspace.QN1[11]*dOld[2] + nmpcWorkspace.QN1[12]*dOld[3] + nmpcWorkspace.QN1[13]*dOld[4] + nmpcWorkspace.QN1[14]*dOld[5] + nmpcWorkspace.QN1[15]*dOld[6] + nmpcWorkspace.QN1[16]*dOld[7] + nmpcWorkspace.QN1[17]*dOld[8];
dNew[2] = + nmpcWorkspace.QN1[18]*dOld[0] + nmpcWorkspace.QN1[19]*dOld[1] + nmpcWorkspace.QN1[20]*dOld[2] + nmpcWorkspace.QN1[21]*dOld[3] + nmpcWorkspace.QN1[22]*dOld[4] + nmpcWorkspace.QN1[23]*dOld[5] + nmpcWorkspace.QN1[24]*dOld[6] + nmpcWorkspace.QN1[25]*dOld[7] + nmpcWorkspace.QN1[26]*dOld[8];
dNew[3] = + nmpcWorkspace.QN1[27]*dOld[0] + nmpcWorkspace.QN1[28]*dOld[1] + nmpcWorkspace.QN1[29]*dOld[2] + nmpcWorkspace.QN1[30]*dOld[3] + nmpcWorkspace.QN1[31]*dOld[4] + nmpcWorkspace.QN1[32]*dOld[5] + nmpcWorkspace.QN1[33]*dOld[6] + nmpcWorkspace.QN1[34]*dOld[7] + nmpcWorkspace.QN1[35]*dOld[8];
dNew[4] = + nmpcWorkspace.QN1[36]*dOld[0] + nmpcWorkspace.QN1[37]*dOld[1] + nmpcWorkspace.QN1[38]*dOld[2] + nmpcWorkspace.QN1[39]*dOld[3] + nmpcWorkspace.QN1[40]*dOld[4] + nmpcWorkspace.QN1[41]*dOld[5] + nmpcWorkspace.QN1[42]*dOld[6] + nmpcWorkspace.QN1[43]*dOld[7] + nmpcWorkspace.QN1[44]*dOld[8];
dNew[5] = + nmpcWorkspace.QN1[45]*dOld[0] + nmpcWorkspace.QN1[46]*dOld[1] + nmpcWorkspace.QN1[47]*dOld[2] + nmpcWorkspace.QN1[48]*dOld[3] + nmpcWorkspace.QN1[49]*dOld[4] + nmpcWorkspace.QN1[50]*dOld[5] + nmpcWorkspace.QN1[51]*dOld[6] + nmpcWorkspace.QN1[52]*dOld[7] + nmpcWorkspace.QN1[53]*dOld[8];
dNew[6] = + nmpcWorkspace.QN1[54]*dOld[0] + nmpcWorkspace.QN1[55]*dOld[1] + nmpcWorkspace.QN1[56]*dOld[2] + nmpcWorkspace.QN1[57]*dOld[3] + nmpcWorkspace.QN1[58]*dOld[4] + nmpcWorkspace.QN1[59]*dOld[5] + nmpcWorkspace.QN1[60]*dOld[6] + nmpcWorkspace.QN1[61]*dOld[7] + nmpcWorkspace.QN1[62]*dOld[8];
dNew[7] = + nmpcWorkspace.QN1[63]*dOld[0] + nmpcWorkspace.QN1[64]*dOld[1] + nmpcWorkspace.QN1[65]*dOld[2] + nmpcWorkspace.QN1[66]*dOld[3] + nmpcWorkspace.QN1[67]*dOld[4] + nmpcWorkspace.QN1[68]*dOld[5] + nmpcWorkspace.QN1[69]*dOld[6] + nmpcWorkspace.QN1[70]*dOld[7] + nmpcWorkspace.QN1[71]*dOld[8];
dNew[8] = + nmpcWorkspace.QN1[72]*dOld[0] + nmpcWorkspace.QN1[73]*dOld[1] + nmpcWorkspace.QN1[74]*dOld[2] + nmpcWorkspace.QN1[75]*dOld[3] + nmpcWorkspace.QN1[76]*dOld[4] + nmpcWorkspace.QN1[77]*dOld[5] + nmpcWorkspace.QN1[78]*dOld[6] + nmpcWorkspace.QN1[79]*dOld[7] + nmpcWorkspace.QN1[80]*dOld[8];
}

void nmpc_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11] + R2[12]*Dy1[12];
RDy1[1] = + R2[13]*Dy1[0] + R2[14]*Dy1[1] + R2[15]*Dy1[2] + R2[16]*Dy1[3] + R2[17]*Dy1[4] + R2[18]*Dy1[5] + R2[19]*Dy1[6] + R2[20]*Dy1[7] + R2[21]*Dy1[8] + R2[22]*Dy1[9] + R2[23]*Dy1[10] + R2[24]*Dy1[11] + R2[25]*Dy1[12];
RDy1[2] = + R2[26]*Dy1[0] + R2[27]*Dy1[1] + R2[28]*Dy1[2] + R2[29]*Dy1[3] + R2[30]*Dy1[4] + R2[31]*Dy1[5] + R2[32]*Dy1[6] + R2[33]*Dy1[7] + R2[34]*Dy1[8] + R2[35]*Dy1[9] + R2[36]*Dy1[10] + R2[37]*Dy1[11] + R2[38]*Dy1[12];
RDy1[3] = + R2[39]*Dy1[0] + R2[40]*Dy1[1] + R2[41]*Dy1[2] + R2[42]*Dy1[3] + R2[43]*Dy1[4] + R2[44]*Dy1[5] + R2[45]*Dy1[6] + R2[46]*Dy1[7] + R2[47]*Dy1[8] + R2[48]*Dy1[9] + R2[49]*Dy1[10] + R2[50]*Dy1[11] + R2[51]*Dy1[12];
}

void nmpc_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11] + Q2[12]*Dy1[12];
QDy1[1] = + Q2[13]*Dy1[0] + Q2[14]*Dy1[1] + Q2[15]*Dy1[2] + Q2[16]*Dy1[3] + Q2[17]*Dy1[4] + Q2[18]*Dy1[5] + Q2[19]*Dy1[6] + Q2[20]*Dy1[7] + Q2[21]*Dy1[8] + Q2[22]*Dy1[9] + Q2[23]*Dy1[10] + Q2[24]*Dy1[11] + Q2[25]*Dy1[12];
QDy1[2] = + Q2[26]*Dy1[0] + Q2[27]*Dy1[1] + Q2[28]*Dy1[2] + Q2[29]*Dy1[3] + Q2[30]*Dy1[4] + Q2[31]*Dy1[5] + Q2[32]*Dy1[6] + Q2[33]*Dy1[7] + Q2[34]*Dy1[8] + Q2[35]*Dy1[9] + Q2[36]*Dy1[10] + Q2[37]*Dy1[11] + Q2[38]*Dy1[12];
QDy1[3] = + Q2[39]*Dy1[0] + Q2[40]*Dy1[1] + Q2[41]*Dy1[2] + Q2[42]*Dy1[3] + Q2[43]*Dy1[4] + Q2[44]*Dy1[5] + Q2[45]*Dy1[6] + Q2[46]*Dy1[7] + Q2[47]*Dy1[8] + Q2[48]*Dy1[9] + Q2[49]*Dy1[10] + Q2[50]*Dy1[11] + Q2[51]*Dy1[12];
QDy1[4] = + Q2[52]*Dy1[0] + Q2[53]*Dy1[1] + Q2[54]*Dy1[2] + Q2[55]*Dy1[3] + Q2[56]*Dy1[4] + Q2[57]*Dy1[5] + Q2[58]*Dy1[6] + Q2[59]*Dy1[7] + Q2[60]*Dy1[8] + Q2[61]*Dy1[9] + Q2[62]*Dy1[10] + Q2[63]*Dy1[11] + Q2[64]*Dy1[12];
QDy1[5] = + Q2[65]*Dy1[0] + Q2[66]*Dy1[1] + Q2[67]*Dy1[2] + Q2[68]*Dy1[3] + Q2[69]*Dy1[4] + Q2[70]*Dy1[5] + Q2[71]*Dy1[6] + Q2[72]*Dy1[7] + Q2[73]*Dy1[8] + Q2[74]*Dy1[9] + Q2[75]*Dy1[10] + Q2[76]*Dy1[11] + Q2[77]*Dy1[12];
QDy1[6] = + Q2[78]*Dy1[0] + Q2[79]*Dy1[1] + Q2[80]*Dy1[2] + Q2[81]*Dy1[3] + Q2[82]*Dy1[4] + Q2[83]*Dy1[5] + Q2[84]*Dy1[6] + Q2[85]*Dy1[7] + Q2[86]*Dy1[8] + Q2[87]*Dy1[9] + Q2[88]*Dy1[10] + Q2[89]*Dy1[11] + Q2[90]*Dy1[12];
QDy1[7] = + Q2[91]*Dy1[0] + Q2[92]*Dy1[1] + Q2[93]*Dy1[2] + Q2[94]*Dy1[3] + Q2[95]*Dy1[4] + Q2[96]*Dy1[5] + Q2[97]*Dy1[6] + Q2[98]*Dy1[7] + Q2[99]*Dy1[8] + Q2[100]*Dy1[9] + Q2[101]*Dy1[10] + Q2[102]*Dy1[11] + Q2[103]*Dy1[12];
QDy1[8] = + Q2[104]*Dy1[0] + Q2[105]*Dy1[1] + Q2[106]*Dy1[2] + Q2[107]*Dy1[3] + Q2[108]*Dy1[4] + Q2[109]*Dy1[5] + Q2[110]*Dy1[6] + Q2[111]*Dy1[7] + Q2[112]*Dy1[8] + Q2[113]*Dy1[9] + Q2[114]*Dy1[10] + Q2[115]*Dy1[11] + Q2[116]*Dy1[12];
}

void nmpc_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[4]*QDy1[1] + E1[8]*QDy1[2] + E1[12]*QDy1[3] + E1[16]*QDy1[4] + E1[20]*QDy1[5] + E1[24]*QDy1[6] + E1[28]*QDy1[7] + E1[32]*QDy1[8];
U1[1] += + E1[1]*QDy1[0] + E1[5]*QDy1[1] + E1[9]*QDy1[2] + E1[13]*QDy1[3] + E1[17]*QDy1[4] + E1[21]*QDy1[5] + E1[25]*QDy1[6] + E1[29]*QDy1[7] + E1[33]*QDy1[8];
U1[2] += + E1[2]*QDy1[0] + E1[6]*QDy1[1] + E1[10]*QDy1[2] + E1[14]*QDy1[3] + E1[18]*QDy1[4] + E1[22]*QDy1[5] + E1[26]*QDy1[6] + E1[30]*QDy1[7] + E1[34]*QDy1[8];
U1[3] += + E1[3]*QDy1[0] + E1[7]*QDy1[1] + E1[11]*QDy1[2] + E1[15]*QDy1[3] + E1[19]*QDy1[4] + E1[23]*QDy1[5] + E1[27]*QDy1[6] + E1[31]*QDy1[7] + E1[35]*QDy1[8];
}

void nmpc_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[4]*Gx1[9] + E1[8]*Gx1[18] + E1[12]*Gx1[27] + E1[16]*Gx1[36] + E1[20]*Gx1[45] + E1[24]*Gx1[54] + E1[28]*Gx1[63] + E1[32]*Gx1[72];
H101[1] += + E1[0]*Gx1[1] + E1[4]*Gx1[10] + E1[8]*Gx1[19] + E1[12]*Gx1[28] + E1[16]*Gx1[37] + E1[20]*Gx1[46] + E1[24]*Gx1[55] + E1[28]*Gx1[64] + E1[32]*Gx1[73];
H101[2] += + E1[0]*Gx1[2] + E1[4]*Gx1[11] + E1[8]*Gx1[20] + E1[12]*Gx1[29] + E1[16]*Gx1[38] + E1[20]*Gx1[47] + E1[24]*Gx1[56] + E1[28]*Gx1[65] + E1[32]*Gx1[74];
H101[3] += + E1[0]*Gx1[3] + E1[4]*Gx1[12] + E1[8]*Gx1[21] + E1[12]*Gx1[30] + E1[16]*Gx1[39] + E1[20]*Gx1[48] + E1[24]*Gx1[57] + E1[28]*Gx1[66] + E1[32]*Gx1[75];
H101[4] += + E1[0]*Gx1[4] + E1[4]*Gx1[13] + E1[8]*Gx1[22] + E1[12]*Gx1[31] + E1[16]*Gx1[40] + E1[20]*Gx1[49] + E1[24]*Gx1[58] + E1[28]*Gx1[67] + E1[32]*Gx1[76];
H101[5] += + E1[0]*Gx1[5] + E1[4]*Gx1[14] + E1[8]*Gx1[23] + E1[12]*Gx1[32] + E1[16]*Gx1[41] + E1[20]*Gx1[50] + E1[24]*Gx1[59] + E1[28]*Gx1[68] + E1[32]*Gx1[77];
H101[6] += + E1[0]*Gx1[6] + E1[4]*Gx1[15] + E1[8]*Gx1[24] + E1[12]*Gx1[33] + E1[16]*Gx1[42] + E1[20]*Gx1[51] + E1[24]*Gx1[60] + E1[28]*Gx1[69] + E1[32]*Gx1[78];
H101[7] += + E1[0]*Gx1[7] + E1[4]*Gx1[16] + E1[8]*Gx1[25] + E1[12]*Gx1[34] + E1[16]*Gx1[43] + E1[20]*Gx1[52] + E1[24]*Gx1[61] + E1[28]*Gx1[70] + E1[32]*Gx1[79];
H101[8] += + E1[0]*Gx1[8] + E1[4]*Gx1[17] + E1[8]*Gx1[26] + E1[12]*Gx1[35] + E1[16]*Gx1[44] + E1[20]*Gx1[53] + E1[24]*Gx1[62] + E1[28]*Gx1[71] + E1[32]*Gx1[80];
H101[9] += + E1[1]*Gx1[0] + E1[5]*Gx1[9] + E1[9]*Gx1[18] + E1[13]*Gx1[27] + E1[17]*Gx1[36] + E1[21]*Gx1[45] + E1[25]*Gx1[54] + E1[29]*Gx1[63] + E1[33]*Gx1[72];
H101[10] += + E1[1]*Gx1[1] + E1[5]*Gx1[10] + E1[9]*Gx1[19] + E1[13]*Gx1[28] + E1[17]*Gx1[37] + E1[21]*Gx1[46] + E1[25]*Gx1[55] + E1[29]*Gx1[64] + E1[33]*Gx1[73];
H101[11] += + E1[1]*Gx1[2] + E1[5]*Gx1[11] + E1[9]*Gx1[20] + E1[13]*Gx1[29] + E1[17]*Gx1[38] + E1[21]*Gx1[47] + E1[25]*Gx1[56] + E1[29]*Gx1[65] + E1[33]*Gx1[74];
H101[12] += + E1[1]*Gx1[3] + E1[5]*Gx1[12] + E1[9]*Gx1[21] + E1[13]*Gx1[30] + E1[17]*Gx1[39] + E1[21]*Gx1[48] + E1[25]*Gx1[57] + E1[29]*Gx1[66] + E1[33]*Gx1[75];
H101[13] += + E1[1]*Gx1[4] + E1[5]*Gx1[13] + E1[9]*Gx1[22] + E1[13]*Gx1[31] + E1[17]*Gx1[40] + E1[21]*Gx1[49] + E1[25]*Gx1[58] + E1[29]*Gx1[67] + E1[33]*Gx1[76];
H101[14] += + E1[1]*Gx1[5] + E1[5]*Gx1[14] + E1[9]*Gx1[23] + E1[13]*Gx1[32] + E1[17]*Gx1[41] + E1[21]*Gx1[50] + E1[25]*Gx1[59] + E1[29]*Gx1[68] + E1[33]*Gx1[77];
H101[15] += + E1[1]*Gx1[6] + E1[5]*Gx1[15] + E1[9]*Gx1[24] + E1[13]*Gx1[33] + E1[17]*Gx1[42] + E1[21]*Gx1[51] + E1[25]*Gx1[60] + E1[29]*Gx1[69] + E1[33]*Gx1[78];
H101[16] += + E1[1]*Gx1[7] + E1[5]*Gx1[16] + E1[9]*Gx1[25] + E1[13]*Gx1[34] + E1[17]*Gx1[43] + E1[21]*Gx1[52] + E1[25]*Gx1[61] + E1[29]*Gx1[70] + E1[33]*Gx1[79];
H101[17] += + E1[1]*Gx1[8] + E1[5]*Gx1[17] + E1[9]*Gx1[26] + E1[13]*Gx1[35] + E1[17]*Gx1[44] + E1[21]*Gx1[53] + E1[25]*Gx1[62] + E1[29]*Gx1[71] + E1[33]*Gx1[80];
H101[18] += + E1[2]*Gx1[0] + E1[6]*Gx1[9] + E1[10]*Gx1[18] + E1[14]*Gx1[27] + E1[18]*Gx1[36] + E1[22]*Gx1[45] + E1[26]*Gx1[54] + E1[30]*Gx1[63] + E1[34]*Gx1[72];
H101[19] += + E1[2]*Gx1[1] + E1[6]*Gx1[10] + E1[10]*Gx1[19] + E1[14]*Gx1[28] + E1[18]*Gx1[37] + E1[22]*Gx1[46] + E1[26]*Gx1[55] + E1[30]*Gx1[64] + E1[34]*Gx1[73];
H101[20] += + E1[2]*Gx1[2] + E1[6]*Gx1[11] + E1[10]*Gx1[20] + E1[14]*Gx1[29] + E1[18]*Gx1[38] + E1[22]*Gx1[47] + E1[26]*Gx1[56] + E1[30]*Gx1[65] + E1[34]*Gx1[74];
H101[21] += + E1[2]*Gx1[3] + E1[6]*Gx1[12] + E1[10]*Gx1[21] + E1[14]*Gx1[30] + E1[18]*Gx1[39] + E1[22]*Gx1[48] + E1[26]*Gx1[57] + E1[30]*Gx1[66] + E1[34]*Gx1[75];
H101[22] += + E1[2]*Gx1[4] + E1[6]*Gx1[13] + E1[10]*Gx1[22] + E1[14]*Gx1[31] + E1[18]*Gx1[40] + E1[22]*Gx1[49] + E1[26]*Gx1[58] + E1[30]*Gx1[67] + E1[34]*Gx1[76];
H101[23] += + E1[2]*Gx1[5] + E1[6]*Gx1[14] + E1[10]*Gx1[23] + E1[14]*Gx1[32] + E1[18]*Gx1[41] + E1[22]*Gx1[50] + E1[26]*Gx1[59] + E1[30]*Gx1[68] + E1[34]*Gx1[77];
H101[24] += + E1[2]*Gx1[6] + E1[6]*Gx1[15] + E1[10]*Gx1[24] + E1[14]*Gx1[33] + E1[18]*Gx1[42] + E1[22]*Gx1[51] + E1[26]*Gx1[60] + E1[30]*Gx1[69] + E1[34]*Gx1[78];
H101[25] += + E1[2]*Gx1[7] + E1[6]*Gx1[16] + E1[10]*Gx1[25] + E1[14]*Gx1[34] + E1[18]*Gx1[43] + E1[22]*Gx1[52] + E1[26]*Gx1[61] + E1[30]*Gx1[70] + E1[34]*Gx1[79];
H101[26] += + E1[2]*Gx1[8] + E1[6]*Gx1[17] + E1[10]*Gx1[26] + E1[14]*Gx1[35] + E1[18]*Gx1[44] + E1[22]*Gx1[53] + E1[26]*Gx1[62] + E1[30]*Gx1[71] + E1[34]*Gx1[80];
H101[27] += + E1[3]*Gx1[0] + E1[7]*Gx1[9] + E1[11]*Gx1[18] + E1[15]*Gx1[27] + E1[19]*Gx1[36] + E1[23]*Gx1[45] + E1[27]*Gx1[54] + E1[31]*Gx1[63] + E1[35]*Gx1[72];
H101[28] += + E1[3]*Gx1[1] + E1[7]*Gx1[10] + E1[11]*Gx1[19] + E1[15]*Gx1[28] + E1[19]*Gx1[37] + E1[23]*Gx1[46] + E1[27]*Gx1[55] + E1[31]*Gx1[64] + E1[35]*Gx1[73];
H101[29] += + E1[3]*Gx1[2] + E1[7]*Gx1[11] + E1[11]*Gx1[20] + E1[15]*Gx1[29] + E1[19]*Gx1[38] + E1[23]*Gx1[47] + E1[27]*Gx1[56] + E1[31]*Gx1[65] + E1[35]*Gx1[74];
H101[30] += + E1[3]*Gx1[3] + E1[7]*Gx1[12] + E1[11]*Gx1[21] + E1[15]*Gx1[30] + E1[19]*Gx1[39] + E1[23]*Gx1[48] + E1[27]*Gx1[57] + E1[31]*Gx1[66] + E1[35]*Gx1[75];
H101[31] += + E1[3]*Gx1[4] + E1[7]*Gx1[13] + E1[11]*Gx1[22] + E1[15]*Gx1[31] + E1[19]*Gx1[40] + E1[23]*Gx1[49] + E1[27]*Gx1[58] + E1[31]*Gx1[67] + E1[35]*Gx1[76];
H101[32] += + E1[3]*Gx1[5] + E1[7]*Gx1[14] + E1[11]*Gx1[23] + E1[15]*Gx1[32] + E1[19]*Gx1[41] + E1[23]*Gx1[50] + E1[27]*Gx1[59] + E1[31]*Gx1[68] + E1[35]*Gx1[77];
H101[33] += + E1[3]*Gx1[6] + E1[7]*Gx1[15] + E1[11]*Gx1[24] + E1[15]*Gx1[33] + E1[19]*Gx1[42] + E1[23]*Gx1[51] + E1[27]*Gx1[60] + E1[31]*Gx1[69] + E1[35]*Gx1[78];
H101[34] += + E1[3]*Gx1[7] + E1[7]*Gx1[16] + E1[11]*Gx1[25] + E1[15]*Gx1[34] + E1[19]*Gx1[43] + E1[23]*Gx1[52] + E1[27]*Gx1[61] + E1[31]*Gx1[70] + E1[35]*Gx1[79];
H101[35] += + E1[3]*Gx1[8] + E1[7]*Gx1[17] + E1[11]*Gx1[26] + E1[15]*Gx1[35] + E1[19]*Gx1[44] + E1[23]*Gx1[53] + E1[27]*Gx1[62] + E1[31]*Gx1[71] + E1[35]*Gx1[80];
}

void nmpc_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 36; lCopy++) H101[ lCopy ] = 0; }
}

void nmpc_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2] + E1[3]*U1[3];
dNew[1] += + E1[4]*U1[0] + E1[5]*U1[1] + E1[6]*U1[2] + E1[7]*U1[3];
dNew[2] += + E1[8]*U1[0] + E1[9]*U1[1] + E1[10]*U1[2] + E1[11]*U1[3];
dNew[3] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2] + E1[15]*U1[3];
dNew[4] += + E1[16]*U1[0] + E1[17]*U1[1] + E1[18]*U1[2] + E1[19]*U1[3];
dNew[5] += + E1[20]*U1[0] + E1[21]*U1[1] + E1[22]*U1[2] + E1[23]*U1[3];
dNew[6] += + E1[24]*U1[0] + E1[25]*U1[1] + E1[26]*U1[2] + E1[27]*U1[3];
dNew[7] += + E1[28]*U1[0] + E1[29]*U1[1] + E1[30]*U1[2] + E1[31]*U1[3];
dNew[8] += + E1[32]*U1[0] + E1[33]*U1[1] + E1[34]*U1[2] + E1[35]*U1[3];
}

void nmpc_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
g1[3] += 0.0;
;
}

void nmpc_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
nmpc_moveGuE( nmpcWorkspace.evGu, nmpcWorkspace.E );
for (lRun1 = 1; lRun1 < 30; ++lRun1)
{
nmpc_moveGxT( &(nmpcWorkspace.evGx[ lRun1 * 81 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ lRun1 * 9-9 ]), &(nmpcWorkspace.evGx[ lRun1 * 81 ]), &(nmpcWorkspace.d[ lRun1 * 9 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ lRun1 * 81-81 ]), &(nmpcWorkspace.evGx[ lRun1 * 81 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ lRun4 * 36 ]), &(nmpcWorkspace.E[ lRun3 * 36 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_moveGuE( &(nmpcWorkspace.evGu[ lRun1 * 36 ]), &(nmpcWorkspace.E[ lRun3 * 36 ]) );
}

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( &(nmpcWorkspace.Q1[ lRun1 * 81 + 81 ]), &(nmpcWorkspace.E[ lRun3 * 36 ]), &(nmpcWorkspace.QE[ lRun3 * 36 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ lRun3 * 36 ]), &(nmpcWorkspace.QE[ lRun3 * 36 ]) );
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ lRun1 * 36 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multQETGx( &(nmpcWorkspace.QE[ lRun3 * 36 ]), &(nmpcWorkspace.evGx[ lRun2 * 81 ]), &(nmpcWorkspace.H10[ lRun1 * 36 ]) );
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_setBlockH11_R1( lRun1, lRun1, &(nmpcWorkspace.R1[ lRun1 * 16 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 36 ]), &(nmpcWorkspace.QE[ lRun5 * 36 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
nmpc_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 36 ]), &(nmpcWorkspace.QE[ lRun5 * 36 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
nmpc_copyHTH( lRun1, lRun2 );
}
}

nmpc_multQ1d( &(nmpcWorkspace.Q1[ 81 ]), nmpcWorkspace.d, nmpcWorkspace.Qd );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 162 ]), &(nmpcWorkspace.d[ 9 ]), &(nmpcWorkspace.Qd[ 9 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 243 ]), &(nmpcWorkspace.d[ 18 ]), &(nmpcWorkspace.Qd[ 18 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 324 ]), &(nmpcWorkspace.d[ 27 ]), &(nmpcWorkspace.Qd[ 27 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 405 ]), &(nmpcWorkspace.d[ 36 ]), &(nmpcWorkspace.Qd[ 36 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 486 ]), &(nmpcWorkspace.d[ 45 ]), &(nmpcWorkspace.Qd[ 45 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 567 ]), &(nmpcWorkspace.d[ 54 ]), &(nmpcWorkspace.Qd[ 54 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 648 ]), &(nmpcWorkspace.d[ 63 ]), &(nmpcWorkspace.Qd[ 63 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 729 ]), &(nmpcWorkspace.d[ 72 ]), &(nmpcWorkspace.Qd[ 72 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 810 ]), &(nmpcWorkspace.d[ 81 ]), &(nmpcWorkspace.Qd[ 81 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 891 ]), &(nmpcWorkspace.d[ 90 ]), &(nmpcWorkspace.Qd[ 90 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 972 ]), &(nmpcWorkspace.d[ 99 ]), &(nmpcWorkspace.Qd[ 99 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1053 ]), &(nmpcWorkspace.d[ 108 ]), &(nmpcWorkspace.Qd[ 108 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1134 ]), &(nmpcWorkspace.d[ 117 ]), &(nmpcWorkspace.Qd[ 117 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1215 ]), &(nmpcWorkspace.d[ 126 ]), &(nmpcWorkspace.Qd[ 126 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1296 ]), &(nmpcWorkspace.d[ 135 ]), &(nmpcWorkspace.Qd[ 135 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1377 ]), &(nmpcWorkspace.d[ 144 ]), &(nmpcWorkspace.Qd[ 144 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1458 ]), &(nmpcWorkspace.d[ 153 ]), &(nmpcWorkspace.Qd[ 153 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1539 ]), &(nmpcWorkspace.d[ 162 ]), &(nmpcWorkspace.Qd[ 162 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1620 ]), &(nmpcWorkspace.d[ 171 ]), &(nmpcWorkspace.Qd[ 171 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1701 ]), &(nmpcWorkspace.d[ 180 ]), &(nmpcWorkspace.Qd[ 180 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1782 ]), &(nmpcWorkspace.d[ 189 ]), &(nmpcWorkspace.Qd[ 189 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1863 ]), &(nmpcWorkspace.d[ 198 ]), &(nmpcWorkspace.Qd[ 198 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1944 ]), &(nmpcWorkspace.d[ 207 ]), &(nmpcWorkspace.Qd[ 207 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2025 ]), &(nmpcWorkspace.d[ 216 ]), &(nmpcWorkspace.Qd[ 216 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2106 ]), &(nmpcWorkspace.d[ 225 ]), &(nmpcWorkspace.Qd[ 225 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2187 ]), &(nmpcWorkspace.d[ 234 ]), &(nmpcWorkspace.Qd[ 234 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2268 ]), &(nmpcWorkspace.d[ 243 ]), &(nmpcWorkspace.Qd[ 243 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2349 ]), &(nmpcWorkspace.d[ 252 ]), &(nmpcWorkspace.Qd[ 252 ]) );
nmpc_multQN1d( nmpcWorkspace.QN1, &(nmpcWorkspace.d[ 261 ]), &(nmpcWorkspace.Qd[ 261 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_macETSlu( &(nmpcWorkspace.QE[ lRun3 * 36 ]), &(nmpcWorkspace.g[ lRun1 * 4 ]) );
}
}
nmpcWorkspace.lb[0] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[0];
nmpcWorkspace.lb[1] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[1];
nmpcWorkspace.lb[2] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[2];
nmpcWorkspace.lb[3] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[3];
nmpcWorkspace.lb[4] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[4];
nmpcWorkspace.lb[5] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[5];
nmpcWorkspace.lb[6] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[6];
nmpcWorkspace.lb[7] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[7];
nmpcWorkspace.lb[8] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[8];
nmpcWorkspace.lb[9] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[9];
nmpcWorkspace.lb[10] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[10];
nmpcWorkspace.lb[11] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[11];
nmpcWorkspace.lb[12] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[12];
nmpcWorkspace.lb[13] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[13];
nmpcWorkspace.lb[14] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[14];
nmpcWorkspace.lb[15] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[15];
nmpcWorkspace.lb[16] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[16];
nmpcWorkspace.lb[17] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[17];
nmpcWorkspace.lb[18] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[18];
nmpcWorkspace.lb[19] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[19];
nmpcWorkspace.lb[20] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[20];
nmpcWorkspace.lb[21] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[21];
nmpcWorkspace.lb[22] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[22];
nmpcWorkspace.lb[23] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[23];
nmpcWorkspace.lb[24] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[24];
nmpcWorkspace.lb[25] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[25];
nmpcWorkspace.lb[26] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[26];
nmpcWorkspace.lb[27] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[27];
nmpcWorkspace.lb[28] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[28];
nmpcWorkspace.lb[29] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[29];
nmpcWorkspace.lb[30] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[30];
nmpcWorkspace.lb[31] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[31];
nmpcWorkspace.lb[32] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[32];
nmpcWorkspace.lb[33] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[33];
nmpcWorkspace.lb[34] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[34];
nmpcWorkspace.lb[35] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[35];
nmpcWorkspace.lb[36] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[36];
nmpcWorkspace.lb[37] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[37];
nmpcWorkspace.lb[38] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[38];
nmpcWorkspace.lb[39] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[39];
nmpcWorkspace.lb[40] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[40];
nmpcWorkspace.lb[41] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[41];
nmpcWorkspace.lb[42] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[42];
nmpcWorkspace.lb[43] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[43];
nmpcWorkspace.lb[44] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[44];
nmpcWorkspace.lb[45] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[45];
nmpcWorkspace.lb[46] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[46];
nmpcWorkspace.lb[47] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[47];
nmpcWorkspace.lb[48] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[48];
nmpcWorkspace.lb[49] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[49];
nmpcWorkspace.lb[50] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[50];
nmpcWorkspace.lb[51] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[51];
nmpcWorkspace.lb[52] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[52];
nmpcWorkspace.lb[53] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[53];
nmpcWorkspace.lb[54] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[54];
nmpcWorkspace.lb[55] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[55];
nmpcWorkspace.lb[56] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[56];
nmpcWorkspace.lb[57] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[57];
nmpcWorkspace.lb[58] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[58];
nmpcWorkspace.lb[59] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[59];
nmpcWorkspace.lb[60] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[60];
nmpcWorkspace.lb[61] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[61];
nmpcWorkspace.lb[62] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[62];
nmpcWorkspace.lb[63] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[63];
nmpcWorkspace.lb[64] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[64];
nmpcWorkspace.lb[65] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[65];
nmpcWorkspace.lb[66] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[66];
nmpcWorkspace.lb[67] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[67];
nmpcWorkspace.lb[68] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[68];
nmpcWorkspace.lb[69] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[69];
nmpcWorkspace.lb[70] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[70];
nmpcWorkspace.lb[71] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[71];
nmpcWorkspace.lb[72] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[72];
nmpcWorkspace.lb[73] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[73];
nmpcWorkspace.lb[74] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[74];
nmpcWorkspace.lb[75] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[75];
nmpcWorkspace.lb[76] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[76];
nmpcWorkspace.lb[77] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[77];
nmpcWorkspace.lb[78] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[78];
nmpcWorkspace.lb[79] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[79];
nmpcWorkspace.lb[80] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[80];
nmpcWorkspace.lb[81] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[81];
nmpcWorkspace.lb[82] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[82];
nmpcWorkspace.lb[83] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[83];
nmpcWorkspace.lb[84] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[84];
nmpcWorkspace.lb[85] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[85];
nmpcWorkspace.lb[86] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[86];
nmpcWorkspace.lb[87] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[87];
nmpcWorkspace.lb[88] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[88];
nmpcWorkspace.lb[89] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[89];
nmpcWorkspace.lb[90] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[90];
nmpcWorkspace.lb[91] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[91];
nmpcWorkspace.lb[92] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[92];
nmpcWorkspace.lb[93] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[93];
nmpcWorkspace.lb[94] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[94];
nmpcWorkspace.lb[95] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[95];
nmpcWorkspace.lb[96] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[96];
nmpcWorkspace.lb[97] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[97];
nmpcWorkspace.lb[98] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[98];
nmpcWorkspace.lb[99] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[99];
nmpcWorkspace.lb[100] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[100];
nmpcWorkspace.lb[101] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[101];
nmpcWorkspace.lb[102] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[102];
nmpcWorkspace.lb[103] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[103];
nmpcWorkspace.lb[104] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[104];
nmpcWorkspace.lb[105] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[105];
nmpcWorkspace.lb[106] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[106];
nmpcWorkspace.lb[107] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[107];
nmpcWorkspace.lb[108] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[108];
nmpcWorkspace.lb[109] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[109];
nmpcWorkspace.lb[110] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[110];
nmpcWorkspace.lb[111] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[111];
nmpcWorkspace.lb[112] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[112];
nmpcWorkspace.lb[113] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[113];
nmpcWorkspace.lb[114] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[114];
nmpcWorkspace.lb[115] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[115];
nmpcWorkspace.lb[116] = (real_t)-1.0000000000000000e+01 - nmpcVariables.u[116];
nmpcWorkspace.lb[117] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[117];
nmpcWorkspace.lb[118] = (real_t)-1.0000000000000001e-01 - nmpcVariables.u[118];
nmpcWorkspace.lb[119] = (real_t)-5.0000000000000000e-01 - nmpcVariables.u[119];
nmpcWorkspace.ub[0] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[0];
nmpcWorkspace.ub[1] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[1];
nmpcWorkspace.ub[2] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[2];
nmpcWorkspace.ub[3] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[3];
nmpcWorkspace.ub[4] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[4];
nmpcWorkspace.ub[5] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[5];
nmpcWorkspace.ub[6] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[6];
nmpcWorkspace.ub[7] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[7];
nmpcWorkspace.ub[8] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[8];
nmpcWorkspace.ub[9] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[9];
nmpcWorkspace.ub[10] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[10];
nmpcWorkspace.ub[11] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[11];
nmpcWorkspace.ub[12] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[12];
nmpcWorkspace.ub[13] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[13];
nmpcWorkspace.ub[14] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[14];
nmpcWorkspace.ub[15] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[15];
nmpcWorkspace.ub[16] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[16];
nmpcWorkspace.ub[17] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[17];
nmpcWorkspace.ub[18] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[18];
nmpcWorkspace.ub[19] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[19];
nmpcWorkspace.ub[20] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[20];
nmpcWorkspace.ub[21] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[21];
nmpcWorkspace.ub[22] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[22];
nmpcWorkspace.ub[23] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[23];
nmpcWorkspace.ub[24] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[24];
nmpcWorkspace.ub[25] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[25];
nmpcWorkspace.ub[26] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[26];
nmpcWorkspace.ub[27] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[27];
nmpcWorkspace.ub[28] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[28];
nmpcWorkspace.ub[29] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[29];
nmpcWorkspace.ub[30] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[30];
nmpcWorkspace.ub[31] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[31];
nmpcWorkspace.ub[32] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[32];
nmpcWorkspace.ub[33] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[33];
nmpcWorkspace.ub[34] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[34];
nmpcWorkspace.ub[35] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[35];
nmpcWorkspace.ub[36] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[36];
nmpcWorkspace.ub[37] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[37];
nmpcWorkspace.ub[38] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[38];
nmpcWorkspace.ub[39] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[39];
nmpcWorkspace.ub[40] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[40];
nmpcWorkspace.ub[41] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[41];
nmpcWorkspace.ub[42] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[42];
nmpcWorkspace.ub[43] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[43];
nmpcWorkspace.ub[44] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[44];
nmpcWorkspace.ub[45] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[45];
nmpcWorkspace.ub[46] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[46];
nmpcWorkspace.ub[47] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[47];
nmpcWorkspace.ub[48] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[48];
nmpcWorkspace.ub[49] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[49];
nmpcWorkspace.ub[50] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[50];
nmpcWorkspace.ub[51] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[51];
nmpcWorkspace.ub[52] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[52];
nmpcWorkspace.ub[53] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[53];
nmpcWorkspace.ub[54] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[54];
nmpcWorkspace.ub[55] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[55];
nmpcWorkspace.ub[56] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[56];
nmpcWorkspace.ub[57] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[57];
nmpcWorkspace.ub[58] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[58];
nmpcWorkspace.ub[59] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[59];
nmpcWorkspace.ub[60] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[60];
nmpcWorkspace.ub[61] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[61];
nmpcWorkspace.ub[62] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[62];
nmpcWorkspace.ub[63] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[63];
nmpcWorkspace.ub[64] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[64];
nmpcWorkspace.ub[65] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[65];
nmpcWorkspace.ub[66] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[66];
nmpcWorkspace.ub[67] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[67];
nmpcWorkspace.ub[68] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[68];
nmpcWorkspace.ub[69] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[69];
nmpcWorkspace.ub[70] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[70];
nmpcWorkspace.ub[71] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[71];
nmpcWorkspace.ub[72] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[72];
nmpcWorkspace.ub[73] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[73];
nmpcWorkspace.ub[74] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[74];
nmpcWorkspace.ub[75] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[75];
nmpcWorkspace.ub[76] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[76];
nmpcWorkspace.ub[77] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[77];
nmpcWorkspace.ub[78] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[78];
nmpcWorkspace.ub[79] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[79];
nmpcWorkspace.ub[80] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[80];
nmpcWorkspace.ub[81] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[81];
nmpcWorkspace.ub[82] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[82];
nmpcWorkspace.ub[83] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[83];
nmpcWorkspace.ub[84] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[84];
nmpcWorkspace.ub[85] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[85];
nmpcWorkspace.ub[86] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[86];
nmpcWorkspace.ub[87] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[87];
nmpcWorkspace.ub[88] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[88];
nmpcWorkspace.ub[89] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[89];
nmpcWorkspace.ub[90] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[90];
nmpcWorkspace.ub[91] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[91];
nmpcWorkspace.ub[92] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[92];
nmpcWorkspace.ub[93] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[93];
nmpcWorkspace.ub[94] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[94];
nmpcWorkspace.ub[95] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[95];
nmpcWorkspace.ub[96] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[96];
nmpcWorkspace.ub[97] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[97];
nmpcWorkspace.ub[98] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[98];
nmpcWorkspace.ub[99] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[99];
nmpcWorkspace.ub[100] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[100];
nmpcWorkspace.ub[101] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[101];
nmpcWorkspace.ub[102] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[102];
nmpcWorkspace.ub[103] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[103];
nmpcWorkspace.ub[104] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[104];
nmpcWorkspace.ub[105] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[105];
nmpcWorkspace.ub[106] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[106];
nmpcWorkspace.ub[107] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[107];
nmpcWorkspace.ub[108] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[108];
nmpcWorkspace.ub[109] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[109];
nmpcWorkspace.ub[110] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[110];
nmpcWorkspace.ub[111] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[111];
nmpcWorkspace.ub[112] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[112];
nmpcWorkspace.ub[113] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[113];
nmpcWorkspace.ub[114] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[114];
nmpcWorkspace.ub[115] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[115];
nmpcWorkspace.ub[116] = (real_t)1.0000000000000000e+01 - nmpcVariables.u[116];
nmpcWorkspace.ub[117] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[117];
nmpcWorkspace.ub[118] = (real_t)1.0000000000000001e-01 - nmpcVariables.u[118];
nmpcWorkspace.ub[119] = (real_t)5.0000000000000000e-01 - nmpcVariables.u[119];

}

void nmpc_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
nmpcWorkspace.Dx0[0] = nmpcVariables.x0[0] - nmpcVariables.x[0];
nmpcWorkspace.Dx0[1] = nmpcVariables.x0[1] - nmpcVariables.x[1];
nmpcWorkspace.Dx0[2] = nmpcVariables.x0[2] - nmpcVariables.x[2];
nmpcWorkspace.Dx0[3] = nmpcVariables.x0[3] - nmpcVariables.x[3];
nmpcWorkspace.Dx0[4] = nmpcVariables.x0[4] - nmpcVariables.x[4];
nmpcWorkspace.Dx0[5] = nmpcVariables.x0[5] - nmpcVariables.x[5];
nmpcWorkspace.Dx0[6] = nmpcVariables.x0[6] - nmpcVariables.x[6];
nmpcWorkspace.Dx0[7] = nmpcVariables.x0[7] - nmpcVariables.x[7];
nmpcWorkspace.Dx0[8] = nmpcVariables.x0[8] - nmpcVariables.x[8];

for (lRun2 = 0; lRun2 < 390; ++lRun2)
nmpcWorkspace.Dy[lRun2] -= nmpcVariables.y[lRun2];

nmpcWorkspace.DyN[0] -= nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] -= nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] -= nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] -= nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] -= nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] -= nmpcVariables.yN[5];
nmpcWorkspace.DyN[6] -= nmpcVariables.yN[6];
nmpcWorkspace.DyN[7] -= nmpcVariables.yN[7];
nmpcWorkspace.DyN[8] -= nmpcVariables.yN[8];

nmpc_multRDy( nmpcWorkspace.R2, nmpcWorkspace.Dy, nmpcWorkspace.g );
nmpc_multRDy( &(nmpcWorkspace.R2[ 52 ]), &(nmpcWorkspace.Dy[ 13 ]), &(nmpcWorkspace.g[ 4 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 104 ]), &(nmpcWorkspace.Dy[ 26 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 156 ]), &(nmpcWorkspace.Dy[ 39 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 208 ]), &(nmpcWorkspace.Dy[ 52 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 260 ]), &(nmpcWorkspace.Dy[ 65 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 312 ]), &(nmpcWorkspace.Dy[ 78 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 364 ]), &(nmpcWorkspace.Dy[ 91 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 416 ]), &(nmpcWorkspace.Dy[ 104 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 468 ]), &(nmpcWorkspace.Dy[ 117 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 520 ]), &(nmpcWorkspace.Dy[ 130 ]), &(nmpcWorkspace.g[ 40 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 572 ]), &(nmpcWorkspace.Dy[ 143 ]), &(nmpcWorkspace.g[ 44 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 624 ]), &(nmpcWorkspace.Dy[ 156 ]), &(nmpcWorkspace.g[ 48 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 676 ]), &(nmpcWorkspace.Dy[ 169 ]), &(nmpcWorkspace.g[ 52 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 728 ]), &(nmpcWorkspace.Dy[ 182 ]), &(nmpcWorkspace.g[ 56 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 780 ]), &(nmpcWorkspace.Dy[ 195 ]), &(nmpcWorkspace.g[ 60 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 832 ]), &(nmpcWorkspace.Dy[ 208 ]), &(nmpcWorkspace.g[ 64 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 884 ]), &(nmpcWorkspace.Dy[ 221 ]), &(nmpcWorkspace.g[ 68 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 936 ]), &(nmpcWorkspace.Dy[ 234 ]), &(nmpcWorkspace.g[ 72 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 988 ]), &(nmpcWorkspace.Dy[ 247 ]), &(nmpcWorkspace.g[ 76 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1040 ]), &(nmpcWorkspace.Dy[ 260 ]), &(nmpcWorkspace.g[ 80 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1092 ]), &(nmpcWorkspace.Dy[ 273 ]), &(nmpcWorkspace.g[ 84 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1144 ]), &(nmpcWorkspace.Dy[ 286 ]), &(nmpcWorkspace.g[ 88 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1196 ]), &(nmpcWorkspace.Dy[ 299 ]), &(nmpcWorkspace.g[ 92 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1248 ]), &(nmpcWorkspace.Dy[ 312 ]), &(nmpcWorkspace.g[ 96 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1300 ]), &(nmpcWorkspace.Dy[ 325 ]), &(nmpcWorkspace.g[ 100 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1352 ]), &(nmpcWorkspace.Dy[ 338 ]), &(nmpcWorkspace.g[ 104 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1404 ]), &(nmpcWorkspace.Dy[ 351 ]), &(nmpcWorkspace.g[ 108 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1456 ]), &(nmpcWorkspace.Dy[ 364 ]), &(nmpcWorkspace.g[ 112 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1508 ]), &(nmpcWorkspace.Dy[ 377 ]), &(nmpcWorkspace.g[ 116 ]) );

nmpc_multQDy( nmpcWorkspace.Q2, nmpcWorkspace.Dy, nmpcWorkspace.QDy );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 117 ]), &(nmpcWorkspace.Dy[ 13 ]), &(nmpcWorkspace.QDy[ 9 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 234 ]), &(nmpcWorkspace.Dy[ 26 ]), &(nmpcWorkspace.QDy[ 18 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 351 ]), &(nmpcWorkspace.Dy[ 39 ]), &(nmpcWorkspace.QDy[ 27 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 468 ]), &(nmpcWorkspace.Dy[ 52 ]), &(nmpcWorkspace.QDy[ 36 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 585 ]), &(nmpcWorkspace.Dy[ 65 ]), &(nmpcWorkspace.QDy[ 45 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 702 ]), &(nmpcWorkspace.Dy[ 78 ]), &(nmpcWorkspace.QDy[ 54 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 819 ]), &(nmpcWorkspace.Dy[ 91 ]), &(nmpcWorkspace.QDy[ 63 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 936 ]), &(nmpcWorkspace.Dy[ 104 ]), &(nmpcWorkspace.QDy[ 72 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1053 ]), &(nmpcWorkspace.Dy[ 117 ]), &(nmpcWorkspace.QDy[ 81 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1170 ]), &(nmpcWorkspace.Dy[ 130 ]), &(nmpcWorkspace.QDy[ 90 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1287 ]), &(nmpcWorkspace.Dy[ 143 ]), &(nmpcWorkspace.QDy[ 99 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1404 ]), &(nmpcWorkspace.Dy[ 156 ]), &(nmpcWorkspace.QDy[ 108 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1521 ]), &(nmpcWorkspace.Dy[ 169 ]), &(nmpcWorkspace.QDy[ 117 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1638 ]), &(nmpcWorkspace.Dy[ 182 ]), &(nmpcWorkspace.QDy[ 126 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1755 ]), &(nmpcWorkspace.Dy[ 195 ]), &(nmpcWorkspace.QDy[ 135 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1872 ]), &(nmpcWorkspace.Dy[ 208 ]), &(nmpcWorkspace.QDy[ 144 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1989 ]), &(nmpcWorkspace.Dy[ 221 ]), &(nmpcWorkspace.QDy[ 153 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2106 ]), &(nmpcWorkspace.Dy[ 234 ]), &(nmpcWorkspace.QDy[ 162 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2223 ]), &(nmpcWorkspace.Dy[ 247 ]), &(nmpcWorkspace.QDy[ 171 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2340 ]), &(nmpcWorkspace.Dy[ 260 ]), &(nmpcWorkspace.QDy[ 180 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2457 ]), &(nmpcWorkspace.Dy[ 273 ]), &(nmpcWorkspace.QDy[ 189 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2574 ]), &(nmpcWorkspace.Dy[ 286 ]), &(nmpcWorkspace.QDy[ 198 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2691 ]), &(nmpcWorkspace.Dy[ 299 ]), &(nmpcWorkspace.QDy[ 207 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2808 ]), &(nmpcWorkspace.Dy[ 312 ]), &(nmpcWorkspace.QDy[ 216 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2925 ]), &(nmpcWorkspace.Dy[ 325 ]), &(nmpcWorkspace.QDy[ 225 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3042 ]), &(nmpcWorkspace.Dy[ 338 ]), &(nmpcWorkspace.QDy[ 234 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3159 ]), &(nmpcWorkspace.Dy[ 351 ]), &(nmpcWorkspace.QDy[ 243 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3276 ]), &(nmpcWorkspace.Dy[ 364 ]), &(nmpcWorkspace.QDy[ 252 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3393 ]), &(nmpcWorkspace.Dy[ 377 ]), &(nmpcWorkspace.QDy[ 261 ]) );

nmpcWorkspace.QDy[270] = + nmpcWorkspace.QN2[0]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[1]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[2]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[3]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[4]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[5]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[6]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[7]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[8]*nmpcWorkspace.DyN[8];
nmpcWorkspace.QDy[271] = + nmpcWorkspace.QN2[9]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[10]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[11]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[12]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[13]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[14]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[15]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[16]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[17]*nmpcWorkspace.DyN[8];
nmpcWorkspace.QDy[272] = + nmpcWorkspace.QN2[18]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[19]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[20]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[21]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[22]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[23]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[24]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[25]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[26]*nmpcWorkspace.DyN[8];
nmpcWorkspace.QDy[273] = + nmpcWorkspace.QN2[27]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[28]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[29]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[30]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[31]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[32]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[33]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[34]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[35]*nmpcWorkspace.DyN[8];
nmpcWorkspace.QDy[274] = + nmpcWorkspace.QN2[36]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[37]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[38]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[39]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[40]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[41]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[42]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[43]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[44]*nmpcWorkspace.DyN[8];
nmpcWorkspace.QDy[275] = + nmpcWorkspace.QN2[45]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[46]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[47]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[48]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[49]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[50]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[51]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[52]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[53]*nmpcWorkspace.DyN[8];
nmpcWorkspace.QDy[276] = + nmpcWorkspace.QN2[54]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[55]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[56]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[57]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[58]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[59]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[60]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[61]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[62]*nmpcWorkspace.DyN[8];
nmpcWorkspace.QDy[277] = + nmpcWorkspace.QN2[63]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[64]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[65]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[66]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[67]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[68]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[69]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[70]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[71]*nmpcWorkspace.DyN[8];
nmpcWorkspace.QDy[278] = + nmpcWorkspace.QN2[72]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[73]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[74]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[75]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[76]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[77]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[78]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[79]*nmpcWorkspace.DyN[7] + nmpcWorkspace.QN2[80]*nmpcWorkspace.DyN[8];

for (lRun2 = 0; lRun2 < 270; ++lRun2)
nmpcWorkspace.QDy[lRun2 + 9] += nmpcWorkspace.Qd[lRun2];


for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multEQDy( &(nmpcWorkspace.E[ lRun3 * 36 ]), &(nmpcWorkspace.QDy[ lRun2 * 9 + 9 ]), &(nmpcWorkspace.g[ lRun1 * 4 ]) );
}
}

nmpcWorkspace.g[0] += + nmpcWorkspace.H10[0]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[1]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[2]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[3]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[4]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[5]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[6]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[7]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[8]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[1] += + nmpcWorkspace.H10[9]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[10]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[11]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[12]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[13]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[14]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[15]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[16]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[17]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[2] += + nmpcWorkspace.H10[18]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[19]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[20]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[21]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[22]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[23]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[24]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[25]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[26]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[3] += + nmpcWorkspace.H10[27]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[28]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[29]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[30]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[31]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[32]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[33]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[34]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[35]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[4] += + nmpcWorkspace.H10[36]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[37]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[38]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[39]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[40]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[41]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[42]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[43]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[44]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[5] += + nmpcWorkspace.H10[45]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[46]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[47]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[48]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[49]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[50]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[51]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[52]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[53]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[6] += + nmpcWorkspace.H10[54]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[55]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[56]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[57]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[58]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[59]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[60]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[61]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[62]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[7] += + nmpcWorkspace.H10[63]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[64]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[65]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[66]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[67]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[68]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[69]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[70]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[71]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[8] += + nmpcWorkspace.H10[72]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[73]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[74]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[75]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[76]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[77]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[78]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[79]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[80]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[9] += + nmpcWorkspace.H10[81]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[82]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[83]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[84]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[85]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[86]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[87]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[88]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[89]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[10] += + nmpcWorkspace.H10[90]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[91]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[92]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[93]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[94]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[95]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[96]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[97]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[98]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[11] += + nmpcWorkspace.H10[99]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[100]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[101]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[102]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[103]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[104]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[105]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[106]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[107]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[12] += + nmpcWorkspace.H10[108]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[109]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[110]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[111]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[112]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[113]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[114]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[115]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[116]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[13] += + nmpcWorkspace.H10[117]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[118]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[119]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[120]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[121]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[122]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[123]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[124]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[125]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[14] += + nmpcWorkspace.H10[126]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[127]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[128]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[129]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[130]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[131]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[132]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[133]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[134]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[15] += + nmpcWorkspace.H10[135]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[136]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[137]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[138]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[139]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[140]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[141]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[142]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[143]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[16] += + nmpcWorkspace.H10[144]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[145]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[146]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[147]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[148]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[149]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[150]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[151]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[152]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[17] += + nmpcWorkspace.H10[153]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[154]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[155]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[156]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[157]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[158]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[159]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[160]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[161]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[18] += + nmpcWorkspace.H10[162]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[163]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[164]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[165]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[166]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[167]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[168]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[169]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[170]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[19] += + nmpcWorkspace.H10[171]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[172]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[173]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[174]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[175]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[176]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[177]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[178]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[179]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[20] += + nmpcWorkspace.H10[180]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[181]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[182]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[183]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[184]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[185]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[186]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[187]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[188]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[21] += + nmpcWorkspace.H10[189]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[190]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[191]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[192]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[193]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[194]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[195]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[196]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[197]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[22] += + nmpcWorkspace.H10[198]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[199]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[200]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[201]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[202]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[203]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[204]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[205]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[206]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[23] += + nmpcWorkspace.H10[207]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[208]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[209]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[210]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[211]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[212]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[213]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[214]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[215]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[24] += + nmpcWorkspace.H10[216]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[217]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[218]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[219]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[220]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[221]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[222]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[223]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[224]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[25] += + nmpcWorkspace.H10[225]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[226]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[227]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[228]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[229]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[230]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[231]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[232]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[233]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[26] += + nmpcWorkspace.H10[234]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[235]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[236]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[237]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[238]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[239]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[240]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[241]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[242]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[27] += + nmpcWorkspace.H10[243]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[244]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[245]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[246]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[247]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[248]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[249]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[250]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[251]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[28] += + nmpcWorkspace.H10[252]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[253]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[254]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[255]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[256]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[257]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[258]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[259]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[260]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[29] += + nmpcWorkspace.H10[261]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[262]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[263]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[264]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[265]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[266]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[267]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[268]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[269]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[30] += + nmpcWorkspace.H10[270]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[271]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[272]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[273]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[274]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[275]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[276]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[277]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[278]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[31] += + nmpcWorkspace.H10[279]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[280]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[281]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[282]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[283]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[284]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[285]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[286]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[287]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[32] += + nmpcWorkspace.H10[288]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[289]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[290]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[291]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[292]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[293]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[294]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[295]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[296]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[33] += + nmpcWorkspace.H10[297]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[298]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[299]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[300]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[301]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[302]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[303]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[304]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[305]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[34] += + nmpcWorkspace.H10[306]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[307]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[308]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[309]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[310]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[311]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[312]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[313]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[314]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[35] += + nmpcWorkspace.H10[315]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[316]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[317]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[318]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[319]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[320]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[321]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[322]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[323]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[36] += + nmpcWorkspace.H10[324]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[325]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[326]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[327]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[328]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[329]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[330]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[331]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[332]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[37] += + nmpcWorkspace.H10[333]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[334]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[335]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[336]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[337]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[338]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[339]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[340]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[341]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[38] += + nmpcWorkspace.H10[342]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[343]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[344]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[345]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[346]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[347]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[348]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[349]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[350]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[39] += + nmpcWorkspace.H10[351]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[352]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[353]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[354]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[355]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[356]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[357]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[358]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[359]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[40] += + nmpcWorkspace.H10[360]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[361]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[362]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[363]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[364]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[365]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[366]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[367]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[368]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[41] += + nmpcWorkspace.H10[369]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[370]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[371]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[372]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[373]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[374]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[375]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[376]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[377]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[42] += + nmpcWorkspace.H10[378]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[379]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[380]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[381]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[382]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[383]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[384]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[385]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[386]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[43] += + nmpcWorkspace.H10[387]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[388]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[389]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[390]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[391]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[392]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[393]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[394]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[395]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[44] += + nmpcWorkspace.H10[396]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[397]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[398]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[399]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[400]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[401]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[402]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[403]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[404]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[45] += + nmpcWorkspace.H10[405]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[406]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[407]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[408]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[409]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[410]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[411]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[412]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[413]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[46] += + nmpcWorkspace.H10[414]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[415]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[416]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[417]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[418]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[419]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[420]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[421]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[422]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[47] += + nmpcWorkspace.H10[423]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[424]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[425]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[426]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[427]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[428]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[429]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[430]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[431]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[48] += + nmpcWorkspace.H10[432]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[433]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[434]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[435]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[436]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[437]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[438]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[439]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[440]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[49] += + nmpcWorkspace.H10[441]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[442]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[443]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[444]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[445]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[446]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[447]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[448]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[449]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[50] += + nmpcWorkspace.H10[450]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[451]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[452]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[453]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[454]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[455]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[456]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[457]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[458]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[51] += + nmpcWorkspace.H10[459]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[460]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[461]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[462]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[463]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[464]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[465]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[466]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[467]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[52] += + nmpcWorkspace.H10[468]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[469]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[470]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[471]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[472]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[473]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[474]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[475]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[476]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[53] += + nmpcWorkspace.H10[477]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[478]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[479]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[480]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[481]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[482]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[483]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[484]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[485]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[54] += + nmpcWorkspace.H10[486]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[487]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[488]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[489]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[490]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[491]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[492]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[493]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[494]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[55] += + nmpcWorkspace.H10[495]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[496]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[497]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[498]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[499]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[500]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[501]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[502]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[503]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[56] += + nmpcWorkspace.H10[504]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[505]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[506]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[507]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[508]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[509]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[510]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[511]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[512]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[57] += + nmpcWorkspace.H10[513]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[514]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[515]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[516]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[517]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[518]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[519]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[520]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[521]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[58] += + nmpcWorkspace.H10[522]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[523]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[524]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[525]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[526]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[527]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[528]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[529]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[530]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[59] += + nmpcWorkspace.H10[531]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[532]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[533]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[534]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[535]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[536]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[537]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[538]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[539]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[60] += + nmpcWorkspace.H10[540]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[541]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[542]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[543]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[544]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[545]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[546]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[547]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[548]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[61] += + nmpcWorkspace.H10[549]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[550]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[551]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[552]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[553]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[554]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[555]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[556]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[557]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[62] += + nmpcWorkspace.H10[558]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[559]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[560]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[561]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[562]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[563]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[564]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[565]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[566]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[63] += + nmpcWorkspace.H10[567]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[568]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[569]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[570]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[571]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[572]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[573]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[574]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[575]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[64] += + nmpcWorkspace.H10[576]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[577]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[578]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[579]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[580]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[581]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[582]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[583]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[584]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[65] += + nmpcWorkspace.H10[585]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[586]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[587]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[588]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[589]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[590]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[591]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[592]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[593]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[66] += + nmpcWorkspace.H10[594]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[595]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[596]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[597]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[598]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[599]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[600]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[601]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[602]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[67] += + nmpcWorkspace.H10[603]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[604]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[605]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[606]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[607]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[608]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[609]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[610]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[611]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[68] += + nmpcWorkspace.H10[612]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[613]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[614]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[615]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[616]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[617]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[618]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[619]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[620]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[69] += + nmpcWorkspace.H10[621]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[622]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[623]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[624]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[625]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[626]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[627]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[628]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[629]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[70] += + nmpcWorkspace.H10[630]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[631]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[632]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[633]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[634]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[635]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[636]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[637]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[638]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[71] += + nmpcWorkspace.H10[639]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[640]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[641]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[642]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[643]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[644]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[645]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[646]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[647]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[72] += + nmpcWorkspace.H10[648]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[649]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[650]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[651]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[652]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[653]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[654]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[655]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[656]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[73] += + nmpcWorkspace.H10[657]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[658]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[659]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[660]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[661]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[662]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[663]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[664]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[665]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[74] += + nmpcWorkspace.H10[666]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[667]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[668]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[669]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[670]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[671]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[672]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[673]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[674]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[75] += + nmpcWorkspace.H10[675]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[676]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[677]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[678]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[679]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[680]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[681]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[682]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[683]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[76] += + nmpcWorkspace.H10[684]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[685]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[686]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[687]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[688]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[689]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[690]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[691]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[692]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[77] += + nmpcWorkspace.H10[693]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[694]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[695]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[696]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[697]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[698]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[699]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[700]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[701]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[78] += + nmpcWorkspace.H10[702]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[703]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[704]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[705]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[706]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[707]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[708]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[709]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[710]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[79] += + nmpcWorkspace.H10[711]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[712]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[713]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[714]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[715]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[716]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[717]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[718]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[719]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[80] += + nmpcWorkspace.H10[720]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[721]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[722]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[723]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[724]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[725]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[726]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[727]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[728]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[81] += + nmpcWorkspace.H10[729]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[730]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[731]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[732]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[733]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[734]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[735]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[736]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[737]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[82] += + nmpcWorkspace.H10[738]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[739]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[740]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[741]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[742]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[743]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[744]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[745]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[746]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[83] += + nmpcWorkspace.H10[747]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[748]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[749]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[750]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[751]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[752]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[753]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[754]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[755]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[84] += + nmpcWorkspace.H10[756]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[757]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[758]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[759]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[760]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[761]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[762]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[763]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[764]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[85] += + nmpcWorkspace.H10[765]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[766]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[767]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[768]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[769]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[770]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[771]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[772]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[773]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[86] += + nmpcWorkspace.H10[774]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[775]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[776]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[777]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[778]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[779]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[780]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[781]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[782]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[87] += + nmpcWorkspace.H10[783]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[784]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[785]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[786]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[787]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[788]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[789]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[790]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[791]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[88] += + nmpcWorkspace.H10[792]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[793]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[794]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[795]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[796]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[797]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[798]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[799]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[800]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[89] += + nmpcWorkspace.H10[801]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[802]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[803]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[804]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[805]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[806]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[807]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[808]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[809]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[90] += + nmpcWorkspace.H10[810]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[811]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[812]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[813]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[814]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[815]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[816]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[817]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[818]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[91] += + nmpcWorkspace.H10[819]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[820]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[821]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[822]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[823]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[824]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[825]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[826]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[827]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[92] += + nmpcWorkspace.H10[828]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[829]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[830]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[831]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[832]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[833]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[834]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[835]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[836]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[93] += + nmpcWorkspace.H10[837]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[838]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[839]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[840]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[841]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[842]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[843]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[844]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[845]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[94] += + nmpcWorkspace.H10[846]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[847]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[848]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[849]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[850]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[851]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[852]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[853]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[854]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[95] += + nmpcWorkspace.H10[855]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[856]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[857]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[858]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[859]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[860]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[861]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[862]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[863]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[96] += + nmpcWorkspace.H10[864]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[865]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[866]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[867]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[868]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[869]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[870]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[871]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[872]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[97] += + nmpcWorkspace.H10[873]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[874]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[875]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[876]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[877]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[878]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[879]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[880]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[881]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[98] += + nmpcWorkspace.H10[882]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[883]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[884]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[885]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[886]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[887]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[888]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[889]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[890]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[99] += + nmpcWorkspace.H10[891]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[892]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[893]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[894]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[895]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[896]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[897]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[898]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[899]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[100] += + nmpcWorkspace.H10[900]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[901]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[902]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[903]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[904]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[905]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[906]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[907]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[908]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[101] += + nmpcWorkspace.H10[909]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[910]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[911]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[912]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[913]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[914]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[915]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[916]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[917]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[102] += + nmpcWorkspace.H10[918]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[919]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[920]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[921]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[922]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[923]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[924]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[925]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[926]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[103] += + nmpcWorkspace.H10[927]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[928]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[929]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[930]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[931]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[932]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[933]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[934]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[935]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[104] += + nmpcWorkspace.H10[936]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[937]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[938]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[939]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[940]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[941]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[942]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[943]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[944]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[105] += + nmpcWorkspace.H10[945]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[946]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[947]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[948]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[949]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[950]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[951]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[952]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[953]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[106] += + nmpcWorkspace.H10[954]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[955]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[956]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[957]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[958]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[959]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[960]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[961]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[962]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[107] += + nmpcWorkspace.H10[963]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[964]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[965]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[966]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[967]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[968]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[969]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[970]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[971]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[108] += + nmpcWorkspace.H10[972]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[973]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[974]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[975]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[976]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[977]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[978]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[979]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[980]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[109] += + nmpcWorkspace.H10[981]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[982]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[983]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[984]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[985]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[986]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[987]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[988]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[989]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[110] += + nmpcWorkspace.H10[990]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[991]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[992]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[993]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[994]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[995]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[996]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[997]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[998]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[111] += + nmpcWorkspace.H10[999]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[1000]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[1001]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[1002]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[1003]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[1004]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[1005]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[1006]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[1007]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[112] += + nmpcWorkspace.H10[1008]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[1009]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[1010]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[1011]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[1012]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[1013]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[1014]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[1015]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[1016]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[113] += + nmpcWorkspace.H10[1017]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[1018]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[1019]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[1020]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[1021]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[1022]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[1023]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[1024]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[1025]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[114] += + nmpcWorkspace.H10[1026]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[1027]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[1028]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[1029]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[1030]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[1031]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[1032]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[1033]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[1034]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[115] += + nmpcWorkspace.H10[1035]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[1036]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[1037]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[1038]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[1039]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[1040]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[1041]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[1042]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[1043]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[116] += + nmpcWorkspace.H10[1044]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[1045]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[1046]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[1047]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[1048]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[1049]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[1050]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[1051]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[1052]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[117] += + nmpcWorkspace.H10[1053]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[1054]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[1055]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[1056]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[1057]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[1058]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[1059]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[1060]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[1061]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[118] += + nmpcWorkspace.H10[1062]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[1063]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[1064]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[1065]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[1066]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[1067]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[1068]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[1069]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[1070]*nmpcWorkspace.Dx0[8];
nmpcWorkspace.g[119] += + nmpcWorkspace.H10[1071]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.H10[1072]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.H10[1073]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.H10[1074]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.H10[1075]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.H10[1076]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.H10[1077]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.H10[1078]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.H10[1079]*nmpcWorkspace.Dx0[8];

}

void nmpc_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
nmpcVariables.u[0] += nmpcWorkspace.x[0];
nmpcVariables.u[1] += nmpcWorkspace.x[1];
nmpcVariables.u[2] += nmpcWorkspace.x[2];
nmpcVariables.u[3] += nmpcWorkspace.x[3];
nmpcVariables.u[4] += nmpcWorkspace.x[4];
nmpcVariables.u[5] += nmpcWorkspace.x[5];
nmpcVariables.u[6] += nmpcWorkspace.x[6];
nmpcVariables.u[7] += nmpcWorkspace.x[7];
nmpcVariables.u[8] += nmpcWorkspace.x[8];
nmpcVariables.u[9] += nmpcWorkspace.x[9];
nmpcVariables.u[10] += nmpcWorkspace.x[10];
nmpcVariables.u[11] += nmpcWorkspace.x[11];
nmpcVariables.u[12] += nmpcWorkspace.x[12];
nmpcVariables.u[13] += nmpcWorkspace.x[13];
nmpcVariables.u[14] += nmpcWorkspace.x[14];
nmpcVariables.u[15] += nmpcWorkspace.x[15];
nmpcVariables.u[16] += nmpcWorkspace.x[16];
nmpcVariables.u[17] += nmpcWorkspace.x[17];
nmpcVariables.u[18] += nmpcWorkspace.x[18];
nmpcVariables.u[19] += nmpcWorkspace.x[19];
nmpcVariables.u[20] += nmpcWorkspace.x[20];
nmpcVariables.u[21] += nmpcWorkspace.x[21];
nmpcVariables.u[22] += nmpcWorkspace.x[22];
nmpcVariables.u[23] += nmpcWorkspace.x[23];
nmpcVariables.u[24] += nmpcWorkspace.x[24];
nmpcVariables.u[25] += nmpcWorkspace.x[25];
nmpcVariables.u[26] += nmpcWorkspace.x[26];
nmpcVariables.u[27] += nmpcWorkspace.x[27];
nmpcVariables.u[28] += nmpcWorkspace.x[28];
nmpcVariables.u[29] += nmpcWorkspace.x[29];
nmpcVariables.u[30] += nmpcWorkspace.x[30];
nmpcVariables.u[31] += nmpcWorkspace.x[31];
nmpcVariables.u[32] += nmpcWorkspace.x[32];
nmpcVariables.u[33] += nmpcWorkspace.x[33];
nmpcVariables.u[34] += nmpcWorkspace.x[34];
nmpcVariables.u[35] += nmpcWorkspace.x[35];
nmpcVariables.u[36] += nmpcWorkspace.x[36];
nmpcVariables.u[37] += nmpcWorkspace.x[37];
nmpcVariables.u[38] += nmpcWorkspace.x[38];
nmpcVariables.u[39] += nmpcWorkspace.x[39];
nmpcVariables.u[40] += nmpcWorkspace.x[40];
nmpcVariables.u[41] += nmpcWorkspace.x[41];
nmpcVariables.u[42] += nmpcWorkspace.x[42];
nmpcVariables.u[43] += nmpcWorkspace.x[43];
nmpcVariables.u[44] += nmpcWorkspace.x[44];
nmpcVariables.u[45] += nmpcWorkspace.x[45];
nmpcVariables.u[46] += nmpcWorkspace.x[46];
nmpcVariables.u[47] += nmpcWorkspace.x[47];
nmpcVariables.u[48] += nmpcWorkspace.x[48];
nmpcVariables.u[49] += nmpcWorkspace.x[49];
nmpcVariables.u[50] += nmpcWorkspace.x[50];
nmpcVariables.u[51] += nmpcWorkspace.x[51];
nmpcVariables.u[52] += nmpcWorkspace.x[52];
nmpcVariables.u[53] += nmpcWorkspace.x[53];
nmpcVariables.u[54] += nmpcWorkspace.x[54];
nmpcVariables.u[55] += nmpcWorkspace.x[55];
nmpcVariables.u[56] += nmpcWorkspace.x[56];
nmpcVariables.u[57] += nmpcWorkspace.x[57];
nmpcVariables.u[58] += nmpcWorkspace.x[58];
nmpcVariables.u[59] += nmpcWorkspace.x[59];
nmpcVariables.u[60] += nmpcWorkspace.x[60];
nmpcVariables.u[61] += nmpcWorkspace.x[61];
nmpcVariables.u[62] += nmpcWorkspace.x[62];
nmpcVariables.u[63] += nmpcWorkspace.x[63];
nmpcVariables.u[64] += nmpcWorkspace.x[64];
nmpcVariables.u[65] += nmpcWorkspace.x[65];
nmpcVariables.u[66] += nmpcWorkspace.x[66];
nmpcVariables.u[67] += nmpcWorkspace.x[67];
nmpcVariables.u[68] += nmpcWorkspace.x[68];
nmpcVariables.u[69] += nmpcWorkspace.x[69];
nmpcVariables.u[70] += nmpcWorkspace.x[70];
nmpcVariables.u[71] += nmpcWorkspace.x[71];
nmpcVariables.u[72] += nmpcWorkspace.x[72];
nmpcVariables.u[73] += nmpcWorkspace.x[73];
nmpcVariables.u[74] += nmpcWorkspace.x[74];
nmpcVariables.u[75] += nmpcWorkspace.x[75];
nmpcVariables.u[76] += nmpcWorkspace.x[76];
nmpcVariables.u[77] += nmpcWorkspace.x[77];
nmpcVariables.u[78] += nmpcWorkspace.x[78];
nmpcVariables.u[79] += nmpcWorkspace.x[79];
nmpcVariables.u[80] += nmpcWorkspace.x[80];
nmpcVariables.u[81] += nmpcWorkspace.x[81];
nmpcVariables.u[82] += nmpcWorkspace.x[82];
nmpcVariables.u[83] += nmpcWorkspace.x[83];
nmpcVariables.u[84] += nmpcWorkspace.x[84];
nmpcVariables.u[85] += nmpcWorkspace.x[85];
nmpcVariables.u[86] += nmpcWorkspace.x[86];
nmpcVariables.u[87] += nmpcWorkspace.x[87];
nmpcVariables.u[88] += nmpcWorkspace.x[88];
nmpcVariables.u[89] += nmpcWorkspace.x[89];
nmpcVariables.u[90] += nmpcWorkspace.x[90];
nmpcVariables.u[91] += nmpcWorkspace.x[91];
nmpcVariables.u[92] += nmpcWorkspace.x[92];
nmpcVariables.u[93] += nmpcWorkspace.x[93];
nmpcVariables.u[94] += nmpcWorkspace.x[94];
nmpcVariables.u[95] += nmpcWorkspace.x[95];
nmpcVariables.u[96] += nmpcWorkspace.x[96];
nmpcVariables.u[97] += nmpcWorkspace.x[97];
nmpcVariables.u[98] += nmpcWorkspace.x[98];
nmpcVariables.u[99] += nmpcWorkspace.x[99];
nmpcVariables.u[100] += nmpcWorkspace.x[100];
nmpcVariables.u[101] += nmpcWorkspace.x[101];
nmpcVariables.u[102] += nmpcWorkspace.x[102];
nmpcVariables.u[103] += nmpcWorkspace.x[103];
nmpcVariables.u[104] += nmpcWorkspace.x[104];
nmpcVariables.u[105] += nmpcWorkspace.x[105];
nmpcVariables.u[106] += nmpcWorkspace.x[106];
nmpcVariables.u[107] += nmpcWorkspace.x[107];
nmpcVariables.u[108] += nmpcWorkspace.x[108];
nmpcVariables.u[109] += nmpcWorkspace.x[109];
nmpcVariables.u[110] += nmpcWorkspace.x[110];
nmpcVariables.u[111] += nmpcWorkspace.x[111];
nmpcVariables.u[112] += nmpcWorkspace.x[112];
nmpcVariables.u[113] += nmpcWorkspace.x[113];
nmpcVariables.u[114] += nmpcWorkspace.x[114];
nmpcVariables.u[115] += nmpcWorkspace.x[115];
nmpcVariables.u[116] += nmpcWorkspace.x[116];
nmpcVariables.u[117] += nmpcWorkspace.x[117];
nmpcVariables.u[118] += nmpcWorkspace.x[118];
nmpcVariables.u[119] += nmpcWorkspace.x[119];

nmpcVariables.x[0] += nmpcWorkspace.Dx0[0];
nmpcVariables.x[1] += nmpcWorkspace.Dx0[1];
nmpcVariables.x[2] += nmpcWorkspace.Dx0[2];
nmpcVariables.x[3] += nmpcWorkspace.Dx0[3];
nmpcVariables.x[4] += nmpcWorkspace.Dx0[4];
nmpcVariables.x[5] += nmpcWorkspace.Dx0[5];
nmpcVariables.x[6] += nmpcWorkspace.Dx0[6];
nmpcVariables.x[7] += nmpcWorkspace.Dx0[7];
nmpcVariables.x[8] += nmpcWorkspace.Dx0[8];

nmpcVariables.x[9] += + nmpcWorkspace.evGx[0]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[3]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[4]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[5]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[6]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[7]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[8]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[0];
nmpcVariables.x[10] += + nmpcWorkspace.evGx[9]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[10]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[11]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[12]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[13]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[14]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[15]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[16]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[17]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[1];
nmpcVariables.x[11] += + nmpcWorkspace.evGx[18]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[19]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[20]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[21]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[22]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[23]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[24]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[25]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[26]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[2];
nmpcVariables.x[12] += + nmpcWorkspace.evGx[27]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[28]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[29]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[30]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[31]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[32]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[33]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[34]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[35]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[3];
nmpcVariables.x[13] += + nmpcWorkspace.evGx[36]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[37]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[38]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[39]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[40]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[41]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[42]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[43]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[44]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[4];
nmpcVariables.x[14] += + nmpcWorkspace.evGx[45]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[46]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[47]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[48]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[49]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[50]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[51]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[52]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[53]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[5];
nmpcVariables.x[15] += + nmpcWorkspace.evGx[54]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[55]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[56]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[57]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[58]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[59]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[60]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[61]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[62]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[6];
nmpcVariables.x[16] += + nmpcWorkspace.evGx[63]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[64]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[65]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[66]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[67]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[68]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[69]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[70]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[71]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[7];
nmpcVariables.x[17] += + nmpcWorkspace.evGx[72]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[73]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[74]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[75]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[76]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[77]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[78]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[79]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[80]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[8];
nmpcVariables.x[18] += + nmpcWorkspace.evGx[81]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[82]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[83]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[84]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[85]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[86]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[87]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[88]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[89]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[9];
nmpcVariables.x[19] += + nmpcWorkspace.evGx[90]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[91]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[92]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[93]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[94]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[95]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[96]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[97]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[98]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[10];
nmpcVariables.x[20] += + nmpcWorkspace.evGx[99]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[100]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[101]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[102]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[103]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[104]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[105]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[106]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[107]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[11];
nmpcVariables.x[21] += + nmpcWorkspace.evGx[108]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[109]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[110]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[111]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[112]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[113]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[114]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[115]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[116]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[12];
nmpcVariables.x[22] += + nmpcWorkspace.evGx[117]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[118]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[119]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[120]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[121]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[122]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[123]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[124]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[125]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[13];
nmpcVariables.x[23] += + nmpcWorkspace.evGx[126]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[127]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[128]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[129]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[130]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[131]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[132]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[133]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[134]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[14];
nmpcVariables.x[24] += + nmpcWorkspace.evGx[135]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[136]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[137]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[138]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[139]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[140]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[141]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[142]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[143]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[15];
nmpcVariables.x[25] += + nmpcWorkspace.evGx[144]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[145]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[146]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[147]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[148]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[149]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[150]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[151]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[152]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[16];
nmpcVariables.x[26] += + nmpcWorkspace.evGx[153]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[154]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[155]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[156]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[157]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[158]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[159]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[160]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[161]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[17];
nmpcVariables.x[27] += + nmpcWorkspace.evGx[162]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[163]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[164]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[165]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[166]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[167]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[168]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[169]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[170]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[18];
nmpcVariables.x[28] += + nmpcWorkspace.evGx[171]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[172]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[173]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[174]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[175]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[176]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[177]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[178]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[179]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[19];
nmpcVariables.x[29] += + nmpcWorkspace.evGx[180]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[181]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[182]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[183]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[184]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[185]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[186]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[187]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[188]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[20];
nmpcVariables.x[30] += + nmpcWorkspace.evGx[189]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[190]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[191]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[192]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[193]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[194]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[195]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[196]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[197]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[21];
nmpcVariables.x[31] += + nmpcWorkspace.evGx[198]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[199]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[200]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[201]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[202]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[203]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[204]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[205]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[206]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[22];
nmpcVariables.x[32] += + nmpcWorkspace.evGx[207]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[208]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[209]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[210]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[211]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[212]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[213]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[214]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[215]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[23];
nmpcVariables.x[33] += + nmpcWorkspace.evGx[216]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[217]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[218]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[219]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[220]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[221]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[222]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[223]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[224]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[24];
nmpcVariables.x[34] += + nmpcWorkspace.evGx[225]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[226]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[227]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[228]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[229]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[230]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[231]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[232]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[233]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[25];
nmpcVariables.x[35] += + nmpcWorkspace.evGx[234]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[235]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[236]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[237]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[238]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[239]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[240]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[241]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[242]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[26];
nmpcVariables.x[36] += + nmpcWorkspace.evGx[243]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[244]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[245]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[246]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[247]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[248]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[249]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[250]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[251]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[27];
nmpcVariables.x[37] += + nmpcWorkspace.evGx[252]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[253]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[254]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[255]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[256]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[257]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[258]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[259]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[260]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[28];
nmpcVariables.x[38] += + nmpcWorkspace.evGx[261]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[262]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[263]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[264]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[265]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[266]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[267]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[268]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[269]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[29];
nmpcVariables.x[39] += + nmpcWorkspace.evGx[270]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[271]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[272]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[273]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[274]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[275]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[276]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[277]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[278]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[30];
nmpcVariables.x[40] += + nmpcWorkspace.evGx[279]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[280]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[281]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[282]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[283]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[284]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[285]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[286]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[287]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[31];
nmpcVariables.x[41] += + nmpcWorkspace.evGx[288]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[289]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[290]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[291]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[292]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[293]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[294]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[295]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[296]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[32];
nmpcVariables.x[42] += + nmpcWorkspace.evGx[297]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[298]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[299]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[300]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[301]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[302]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[303]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[304]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[305]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[33];
nmpcVariables.x[43] += + nmpcWorkspace.evGx[306]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[307]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[308]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[309]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[310]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[311]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[312]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[313]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[314]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[34];
nmpcVariables.x[44] += + nmpcWorkspace.evGx[315]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[316]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[317]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[318]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[319]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[320]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[321]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[322]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[323]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[35];
nmpcVariables.x[45] += + nmpcWorkspace.evGx[324]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[325]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[326]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[327]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[328]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[329]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[330]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[331]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[332]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[36];
nmpcVariables.x[46] += + nmpcWorkspace.evGx[333]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[334]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[335]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[336]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[337]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[338]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[339]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[340]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[341]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[37];
nmpcVariables.x[47] += + nmpcWorkspace.evGx[342]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[343]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[344]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[345]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[346]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[347]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[348]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[349]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[350]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[38];
nmpcVariables.x[48] += + nmpcWorkspace.evGx[351]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[352]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[353]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[354]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[355]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[356]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[357]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[358]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[359]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[39];
nmpcVariables.x[49] += + nmpcWorkspace.evGx[360]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[361]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[362]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[363]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[364]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[365]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[366]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[367]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[368]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[40];
nmpcVariables.x[50] += + nmpcWorkspace.evGx[369]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[370]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[371]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[372]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[373]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[374]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[375]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[376]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[377]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[41];
nmpcVariables.x[51] += + nmpcWorkspace.evGx[378]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[379]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[380]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[381]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[382]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[383]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[384]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[385]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[386]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[42];
nmpcVariables.x[52] += + nmpcWorkspace.evGx[387]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[388]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[389]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[390]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[391]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[392]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[393]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[394]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[395]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[43];
nmpcVariables.x[53] += + nmpcWorkspace.evGx[396]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[397]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[398]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[399]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[400]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[401]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[402]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[403]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[404]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[44];
nmpcVariables.x[54] += + nmpcWorkspace.evGx[405]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[406]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[407]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[408]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[409]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[410]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[411]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[412]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[413]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[45];
nmpcVariables.x[55] += + nmpcWorkspace.evGx[414]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[415]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[416]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[417]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[418]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[419]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[420]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[421]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[422]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[46];
nmpcVariables.x[56] += + nmpcWorkspace.evGx[423]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[424]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[425]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[426]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[427]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[428]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[429]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[430]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[431]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[47];
nmpcVariables.x[57] += + nmpcWorkspace.evGx[432]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[433]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[434]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[435]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[436]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[437]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[438]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[439]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[440]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[48];
nmpcVariables.x[58] += + nmpcWorkspace.evGx[441]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[442]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[443]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[444]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[445]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[446]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[447]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[448]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[449]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[49];
nmpcVariables.x[59] += + nmpcWorkspace.evGx[450]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[451]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[452]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[453]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[454]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[455]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[456]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[457]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[458]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[50];
nmpcVariables.x[60] += + nmpcWorkspace.evGx[459]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[460]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[461]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[462]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[463]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[464]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[465]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[466]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[467]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[51];
nmpcVariables.x[61] += + nmpcWorkspace.evGx[468]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[469]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[470]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[471]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[472]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[473]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[474]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[475]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[476]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[52];
nmpcVariables.x[62] += + nmpcWorkspace.evGx[477]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[478]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[479]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[480]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[481]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[482]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[483]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[484]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[485]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[53];
nmpcVariables.x[63] += + nmpcWorkspace.evGx[486]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[487]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[488]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[489]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[490]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[491]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[492]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[493]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[494]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[54];
nmpcVariables.x[64] += + nmpcWorkspace.evGx[495]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[496]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[497]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[498]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[499]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[500]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[501]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[502]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[503]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[55];
nmpcVariables.x[65] += + nmpcWorkspace.evGx[504]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[505]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[506]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[507]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[508]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[509]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[510]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[511]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[512]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[56];
nmpcVariables.x[66] += + nmpcWorkspace.evGx[513]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[514]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[515]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[516]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[517]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[518]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[519]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[520]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[521]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[57];
nmpcVariables.x[67] += + nmpcWorkspace.evGx[522]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[523]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[524]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[525]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[526]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[527]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[528]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[529]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[530]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[58];
nmpcVariables.x[68] += + nmpcWorkspace.evGx[531]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[532]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[533]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[534]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[535]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[536]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[537]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[538]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[539]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[59];
nmpcVariables.x[69] += + nmpcWorkspace.evGx[540]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[541]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[542]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[543]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[544]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[545]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[546]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[547]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[548]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[60];
nmpcVariables.x[70] += + nmpcWorkspace.evGx[549]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[550]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[551]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[552]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[553]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[554]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[555]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[556]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[557]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[61];
nmpcVariables.x[71] += + nmpcWorkspace.evGx[558]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[559]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[560]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[561]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[562]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[563]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[564]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[565]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[566]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[62];
nmpcVariables.x[72] += + nmpcWorkspace.evGx[567]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[568]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[569]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[570]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[571]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[572]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[573]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[574]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[575]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[63];
nmpcVariables.x[73] += + nmpcWorkspace.evGx[576]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[577]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[578]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[579]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[580]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[581]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[582]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[583]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[584]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[64];
nmpcVariables.x[74] += + nmpcWorkspace.evGx[585]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[586]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[587]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[588]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[589]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[590]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[591]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[592]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[593]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[65];
nmpcVariables.x[75] += + nmpcWorkspace.evGx[594]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[595]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[596]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[597]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[598]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[599]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[600]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[601]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[602]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[66];
nmpcVariables.x[76] += + nmpcWorkspace.evGx[603]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[604]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[605]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[606]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[607]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[608]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[609]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[610]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[611]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[67];
nmpcVariables.x[77] += + nmpcWorkspace.evGx[612]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[613]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[614]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[615]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[616]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[617]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[618]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[619]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[620]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[68];
nmpcVariables.x[78] += + nmpcWorkspace.evGx[621]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[622]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[623]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[624]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[625]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[626]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[627]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[628]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[629]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[69];
nmpcVariables.x[79] += + nmpcWorkspace.evGx[630]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[631]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[632]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[633]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[634]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[635]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[636]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[637]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[638]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[70];
nmpcVariables.x[80] += + nmpcWorkspace.evGx[639]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[640]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[641]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[642]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[643]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[644]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[645]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[646]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[647]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[71];
nmpcVariables.x[81] += + nmpcWorkspace.evGx[648]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[649]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[650]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[651]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[652]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[653]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[654]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[655]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[656]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[72];
nmpcVariables.x[82] += + nmpcWorkspace.evGx[657]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[658]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[659]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[660]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[661]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[662]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[663]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[664]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[665]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[73];
nmpcVariables.x[83] += + nmpcWorkspace.evGx[666]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[667]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[668]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[669]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[670]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[671]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[672]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[673]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[674]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[74];
nmpcVariables.x[84] += + nmpcWorkspace.evGx[675]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[676]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[677]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[678]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[679]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[680]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[681]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[682]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[683]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[75];
nmpcVariables.x[85] += + nmpcWorkspace.evGx[684]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[685]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[686]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[687]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[688]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[689]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[690]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[691]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[692]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[76];
nmpcVariables.x[86] += + nmpcWorkspace.evGx[693]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[694]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[695]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[696]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[697]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[698]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[699]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[700]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[701]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[77];
nmpcVariables.x[87] += + nmpcWorkspace.evGx[702]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[703]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[704]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[705]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[706]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[707]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[708]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[709]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[710]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[78];
nmpcVariables.x[88] += + nmpcWorkspace.evGx[711]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[712]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[713]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[714]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[715]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[716]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[717]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[718]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[719]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[79];
nmpcVariables.x[89] += + nmpcWorkspace.evGx[720]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[721]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[722]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[723]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[724]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[725]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[726]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[727]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[728]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[80];
nmpcVariables.x[90] += + nmpcWorkspace.evGx[729]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[730]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[731]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[732]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[733]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[734]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[735]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[736]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[737]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[81];
nmpcVariables.x[91] += + nmpcWorkspace.evGx[738]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[739]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[740]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[741]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[742]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[743]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[744]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[745]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[746]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[82];
nmpcVariables.x[92] += + nmpcWorkspace.evGx[747]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[748]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[749]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[750]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[751]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[752]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[753]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[754]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[755]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[83];
nmpcVariables.x[93] += + nmpcWorkspace.evGx[756]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[757]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[758]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[759]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[760]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[761]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[762]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[763]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[764]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[84];
nmpcVariables.x[94] += + nmpcWorkspace.evGx[765]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[766]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[767]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[768]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[769]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[770]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[771]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[772]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[773]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[85];
nmpcVariables.x[95] += + nmpcWorkspace.evGx[774]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[775]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[776]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[777]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[778]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[779]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[780]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[781]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[782]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[86];
nmpcVariables.x[96] += + nmpcWorkspace.evGx[783]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[784]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[785]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[786]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[787]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[788]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[789]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[790]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[791]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[87];
nmpcVariables.x[97] += + nmpcWorkspace.evGx[792]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[793]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[794]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[795]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[796]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[797]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[798]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[799]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[800]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[88];
nmpcVariables.x[98] += + nmpcWorkspace.evGx[801]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[802]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[803]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[804]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[805]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[806]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[807]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[808]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[809]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[89];
nmpcVariables.x[99] += + nmpcWorkspace.evGx[810]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[811]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[812]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[813]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[814]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[815]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[816]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[817]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[818]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[90];
nmpcVariables.x[100] += + nmpcWorkspace.evGx[819]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[820]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[821]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[822]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[823]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[824]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[825]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[826]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[827]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[91];
nmpcVariables.x[101] += + nmpcWorkspace.evGx[828]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[829]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[830]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[831]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[832]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[833]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[834]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[835]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[836]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[92];
nmpcVariables.x[102] += + nmpcWorkspace.evGx[837]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[838]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[839]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[840]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[841]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[842]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[843]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[844]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[845]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[93];
nmpcVariables.x[103] += + nmpcWorkspace.evGx[846]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[847]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[848]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[849]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[850]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[851]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[852]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[853]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[854]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[94];
nmpcVariables.x[104] += + nmpcWorkspace.evGx[855]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[856]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[857]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[858]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[859]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[860]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[861]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[862]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[863]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[95];
nmpcVariables.x[105] += + nmpcWorkspace.evGx[864]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[865]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[866]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[867]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[868]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[869]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[870]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[871]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[872]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[96];
nmpcVariables.x[106] += + nmpcWorkspace.evGx[873]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[874]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[875]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[876]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[877]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[878]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[879]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[880]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[881]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[97];
nmpcVariables.x[107] += + nmpcWorkspace.evGx[882]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[883]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[884]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[885]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[886]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[887]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[888]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[889]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[890]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[98];
nmpcVariables.x[108] += + nmpcWorkspace.evGx[891]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[892]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[893]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[894]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[895]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[896]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[897]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[898]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[899]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[99];
nmpcVariables.x[109] += + nmpcWorkspace.evGx[900]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[901]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[902]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[903]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[904]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[905]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[906]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[907]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[908]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[100];
nmpcVariables.x[110] += + nmpcWorkspace.evGx[909]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[910]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[911]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[912]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[913]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[914]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[915]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[916]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[917]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[101];
nmpcVariables.x[111] += + nmpcWorkspace.evGx[918]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[919]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[920]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[921]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[922]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[923]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[924]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[925]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[926]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[102];
nmpcVariables.x[112] += + nmpcWorkspace.evGx[927]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[928]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[929]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[930]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[931]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[932]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[933]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[934]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[935]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[103];
nmpcVariables.x[113] += + nmpcWorkspace.evGx[936]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[937]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[938]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[939]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[940]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[941]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[942]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[943]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[944]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[104];
nmpcVariables.x[114] += + nmpcWorkspace.evGx[945]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[946]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[947]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[948]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[949]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[950]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[951]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[952]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[953]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[105];
nmpcVariables.x[115] += + nmpcWorkspace.evGx[954]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[955]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[956]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[957]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[958]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[959]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[960]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[961]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[962]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[106];
nmpcVariables.x[116] += + nmpcWorkspace.evGx[963]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[964]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[965]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[966]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[967]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[968]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[969]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[970]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[971]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[107];
nmpcVariables.x[117] += + nmpcWorkspace.evGx[972]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[973]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[974]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[975]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[976]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[977]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[978]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[979]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[980]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[108];
nmpcVariables.x[118] += + nmpcWorkspace.evGx[981]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[982]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[983]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[984]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[985]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[986]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[987]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[988]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[989]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[109];
nmpcVariables.x[119] += + nmpcWorkspace.evGx[990]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[991]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[992]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[993]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[994]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[995]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[996]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[997]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[998]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[110];
nmpcVariables.x[120] += + nmpcWorkspace.evGx[999]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1000]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1001]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1002]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1003]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1004]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1005]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1006]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1007]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[111];
nmpcVariables.x[121] += + nmpcWorkspace.evGx[1008]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1009]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1010]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1011]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1012]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1013]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1014]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1015]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1016]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[112];
nmpcVariables.x[122] += + nmpcWorkspace.evGx[1017]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1018]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1019]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1020]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1021]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1022]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1023]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1024]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1025]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[113];
nmpcVariables.x[123] += + nmpcWorkspace.evGx[1026]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1027]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1028]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1029]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1030]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1031]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1032]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1033]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1034]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[114];
nmpcVariables.x[124] += + nmpcWorkspace.evGx[1035]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1036]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1037]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1038]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1039]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1040]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1041]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1042]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1043]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[115];
nmpcVariables.x[125] += + nmpcWorkspace.evGx[1044]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1045]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1046]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1047]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1048]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1049]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1050]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1051]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1052]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[116];
nmpcVariables.x[126] += + nmpcWorkspace.evGx[1053]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1054]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1055]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1056]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1057]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1058]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1059]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1060]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1061]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[117];
nmpcVariables.x[127] += + nmpcWorkspace.evGx[1062]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1063]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1064]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1065]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1066]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1067]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1068]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1069]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1070]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[118];
nmpcVariables.x[128] += + nmpcWorkspace.evGx[1071]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1072]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1073]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1074]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1075]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1076]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1077]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1078]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1079]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[119];
nmpcVariables.x[129] += + nmpcWorkspace.evGx[1080]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1081]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1082]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1083]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1084]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1085]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1086]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1087]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1088]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[120];
nmpcVariables.x[130] += + nmpcWorkspace.evGx[1089]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1090]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1091]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1092]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1093]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1094]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1095]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1096]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1097]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[121];
nmpcVariables.x[131] += + nmpcWorkspace.evGx[1098]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1099]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1100]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1101]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1102]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1103]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1104]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1105]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1106]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[122];
nmpcVariables.x[132] += + nmpcWorkspace.evGx[1107]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1108]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1109]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1110]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1111]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1112]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1113]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1114]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1115]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[123];
nmpcVariables.x[133] += + nmpcWorkspace.evGx[1116]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1117]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1118]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1119]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1120]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1121]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1122]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1123]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1124]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[124];
nmpcVariables.x[134] += + nmpcWorkspace.evGx[1125]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1126]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1127]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1128]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1129]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1130]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1131]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1132]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1133]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[125];
nmpcVariables.x[135] += + nmpcWorkspace.evGx[1134]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1135]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1136]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1137]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1138]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1139]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1140]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1141]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1142]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[126];
nmpcVariables.x[136] += + nmpcWorkspace.evGx[1143]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1144]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1145]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1146]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1147]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1148]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1149]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1150]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1151]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[127];
nmpcVariables.x[137] += + nmpcWorkspace.evGx[1152]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1153]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1154]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1155]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1156]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1157]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1158]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1159]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1160]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[128];
nmpcVariables.x[138] += + nmpcWorkspace.evGx[1161]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1162]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1163]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1164]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1165]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1166]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1167]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1168]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1169]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[129];
nmpcVariables.x[139] += + nmpcWorkspace.evGx[1170]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1171]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1172]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1173]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1174]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1175]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1176]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1177]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1178]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[130];
nmpcVariables.x[140] += + nmpcWorkspace.evGx[1179]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1180]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1181]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1182]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1183]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1184]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1185]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1186]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1187]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[131];
nmpcVariables.x[141] += + nmpcWorkspace.evGx[1188]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1189]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1190]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1191]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1192]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1193]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1194]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1195]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1196]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[132];
nmpcVariables.x[142] += + nmpcWorkspace.evGx[1197]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1198]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1199]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1200]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1201]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1202]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1203]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1204]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1205]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[133];
nmpcVariables.x[143] += + nmpcWorkspace.evGx[1206]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1207]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1208]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1209]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1210]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1211]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1212]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1213]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1214]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[134];
nmpcVariables.x[144] += + nmpcWorkspace.evGx[1215]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1216]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1217]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1218]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1219]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1220]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1221]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1222]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1223]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[135];
nmpcVariables.x[145] += + nmpcWorkspace.evGx[1224]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1225]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1226]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1227]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1228]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1229]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1230]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1231]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1232]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[136];
nmpcVariables.x[146] += + nmpcWorkspace.evGx[1233]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1234]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1235]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1236]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1237]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1238]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1239]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1240]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1241]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[137];
nmpcVariables.x[147] += + nmpcWorkspace.evGx[1242]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1243]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1244]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1245]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1246]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1247]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1248]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1249]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1250]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[138];
nmpcVariables.x[148] += + nmpcWorkspace.evGx[1251]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1252]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1253]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1254]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1255]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1256]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1257]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1258]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1259]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[139];
nmpcVariables.x[149] += + nmpcWorkspace.evGx[1260]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1261]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1262]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1263]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1264]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1265]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1266]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1267]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1268]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[140];
nmpcVariables.x[150] += + nmpcWorkspace.evGx[1269]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1270]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1271]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1272]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1273]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1274]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1275]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1276]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1277]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[141];
nmpcVariables.x[151] += + nmpcWorkspace.evGx[1278]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1279]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1280]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1281]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1282]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1283]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1284]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1285]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1286]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[142];
nmpcVariables.x[152] += + nmpcWorkspace.evGx[1287]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1288]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1289]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1290]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1291]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1292]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1293]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1294]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1295]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[143];
nmpcVariables.x[153] += + nmpcWorkspace.evGx[1296]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1297]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1298]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1299]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1300]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1301]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1302]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1303]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1304]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[144];
nmpcVariables.x[154] += + nmpcWorkspace.evGx[1305]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1306]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1307]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1308]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1309]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1310]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1311]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1312]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1313]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[145];
nmpcVariables.x[155] += + nmpcWorkspace.evGx[1314]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1315]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1316]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1317]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1318]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1319]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1320]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1321]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1322]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[146];
nmpcVariables.x[156] += + nmpcWorkspace.evGx[1323]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1324]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1325]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1326]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1327]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1328]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1329]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1330]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1331]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[147];
nmpcVariables.x[157] += + nmpcWorkspace.evGx[1332]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1333]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1334]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1335]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1336]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1337]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1338]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1339]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1340]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[148];
nmpcVariables.x[158] += + nmpcWorkspace.evGx[1341]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1342]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1343]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1344]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1345]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1346]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1347]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1348]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1349]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[149];
nmpcVariables.x[159] += + nmpcWorkspace.evGx[1350]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1351]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1352]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1353]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1354]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1355]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1356]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1357]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1358]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[150];
nmpcVariables.x[160] += + nmpcWorkspace.evGx[1359]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1360]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1361]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1362]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1363]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1364]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1365]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1366]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1367]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[151];
nmpcVariables.x[161] += + nmpcWorkspace.evGx[1368]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1369]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1370]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1371]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1372]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1373]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1374]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1375]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1376]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[152];
nmpcVariables.x[162] += + nmpcWorkspace.evGx[1377]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1378]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1379]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1380]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1381]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1382]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1383]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1384]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1385]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[153];
nmpcVariables.x[163] += + nmpcWorkspace.evGx[1386]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1387]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1388]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1389]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1390]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1391]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1392]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1393]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1394]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[154];
nmpcVariables.x[164] += + nmpcWorkspace.evGx[1395]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1396]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1397]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1398]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1399]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1400]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1401]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1402]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1403]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[155];
nmpcVariables.x[165] += + nmpcWorkspace.evGx[1404]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1405]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1406]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1407]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1408]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1409]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1410]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1411]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1412]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[156];
nmpcVariables.x[166] += + nmpcWorkspace.evGx[1413]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1414]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1415]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1416]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1417]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1418]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1419]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1420]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1421]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[157];
nmpcVariables.x[167] += + nmpcWorkspace.evGx[1422]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1423]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1424]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1425]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1426]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1427]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1428]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1429]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1430]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[158];
nmpcVariables.x[168] += + nmpcWorkspace.evGx[1431]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1432]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1433]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1434]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1435]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1436]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1437]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1438]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1439]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[159];
nmpcVariables.x[169] += + nmpcWorkspace.evGx[1440]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1441]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1442]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1443]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1444]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1445]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1446]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1447]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1448]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[160];
nmpcVariables.x[170] += + nmpcWorkspace.evGx[1449]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1450]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1451]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1452]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1453]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1454]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1455]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1456]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1457]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[161];
nmpcVariables.x[171] += + nmpcWorkspace.evGx[1458]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1459]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1460]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1461]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1462]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1463]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1464]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1465]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1466]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[162];
nmpcVariables.x[172] += + nmpcWorkspace.evGx[1467]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1468]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1469]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1470]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1471]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1472]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1473]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1474]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1475]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[163];
nmpcVariables.x[173] += + nmpcWorkspace.evGx[1476]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1477]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1478]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1479]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1480]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1481]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1482]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1483]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1484]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[164];
nmpcVariables.x[174] += + nmpcWorkspace.evGx[1485]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1486]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1487]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1488]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1489]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1490]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1491]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1492]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1493]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[165];
nmpcVariables.x[175] += + nmpcWorkspace.evGx[1494]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1495]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1496]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1497]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1498]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1499]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1500]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1501]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1502]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[166];
nmpcVariables.x[176] += + nmpcWorkspace.evGx[1503]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1504]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1505]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1506]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1507]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1508]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1509]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1510]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1511]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[167];
nmpcVariables.x[177] += + nmpcWorkspace.evGx[1512]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1513]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1514]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1515]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1516]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1517]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1518]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1519]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1520]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[168];
nmpcVariables.x[178] += + nmpcWorkspace.evGx[1521]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1522]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1523]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1524]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1525]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1526]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1527]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1528]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1529]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[169];
nmpcVariables.x[179] += + nmpcWorkspace.evGx[1530]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1531]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1532]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1533]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1534]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1535]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1536]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1537]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1538]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[170];
nmpcVariables.x[180] += + nmpcWorkspace.evGx[1539]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1540]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1541]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1542]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1543]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1544]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1545]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1546]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1547]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[171];
nmpcVariables.x[181] += + nmpcWorkspace.evGx[1548]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1549]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1550]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1551]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1552]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1553]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1554]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1555]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1556]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[172];
nmpcVariables.x[182] += + nmpcWorkspace.evGx[1557]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1558]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1559]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1560]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1561]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1562]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1563]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1564]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1565]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[173];
nmpcVariables.x[183] += + nmpcWorkspace.evGx[1566]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1567]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1568]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1569]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1570]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1571]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1572]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1573]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1574]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[174];
nmpcVariables.x[184] += + nmpcWorkspace.evGx[1575]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1576]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1577]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1578]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1579]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1580]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1581]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1582]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1583]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[175];
nmpcVariables.x[185] += + nmpcWorkspace.evGx[1584]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1585]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1586]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1587]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1588]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1589]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1590]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1591]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1592]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[176];
nmpcVariables.x[186] += + nmpcWorkspace.evGx[1593]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1594]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1595]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1596]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1597]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1598]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1599]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1600]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1601]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[177];
nmpcVariables.x[187] += + nmpcWorkspace.evGx[1602]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1603]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1604]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1605]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1606]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1607]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1608]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1609]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1610]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[178];
nmpcVariables.x[188] += + nmpcWorkspace.evGx[1611]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1612]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1613]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1614]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1615]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1616]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1617]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1618]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1619]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[179];
nmpcVariables.x[189] += + nmpcWorkspace.evGx[1620]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1621]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1622]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1623]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1624]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1625]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1626]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1627]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1628]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[180];
nmpcVariables.x[190] += + nmpcWorkspace.evGx[1629]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1630]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1631]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1632]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1633]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1634]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1635]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1636]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1637]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[181];
nmpcVariables.x[191] += + nmpcWorkspace.evGx[1638]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1639]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1640]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1641]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1642]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1643]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1644]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1645]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1646]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[182];
nmpcVariables.x[192] += + nmpcWorkspace.evGx[1647]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1648]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1649]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1650]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1651]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1652]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1653]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1654]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1655]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[183];
nmpcVariables.x[193] += + nmpcWorkspace.evGx[1656]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1657]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1658]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1659]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1660]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1661]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1662]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1663]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1664]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[184];
nmpcVariables.x[194] += + nmpcWorkspace.evGx[1665]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1666]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1667]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1668]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1669]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1670]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1671]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1672]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1673]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[185];
nmpcVariables.x[195] += + nmpcWorkspace.evGx[1674]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1675]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1676]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1677]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1678]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1679]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1680]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1681]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1682]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[186];
nmpcVariables.x[196] += + nmpcWorkspace.evGx[1683]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1684]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1685]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1686]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1687]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1688]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1689]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1690]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1691]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[187];
nmpcVariables.x[197] += + nmpcWorkspace.evGx[1692]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1693]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1694]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1695]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1696]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1697]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1698]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1699]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1700]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[188];
nmpcVariables.x[198] += + nmpcWorkspace.evGx[1701]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1702]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1703]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1704]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1705]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1706]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1707]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1708]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1709]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[189];
nmpcVariables.x[199] += + nmpcWorkspace.evGx[1710]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1711]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1712]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1713]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1714]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1715]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1716]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1717]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1718]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[190];
nmpcVariables.x[200] += + nmpcWorkspace.evGx[1719]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1720]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1721]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1722]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1723]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1724]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1725]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1726]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1727]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[191];
nmpcVariables.x[201] += + nmpcWorkspace.evGx[1728]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1729]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1730]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1731]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1732]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1733]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1734]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1735]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1736]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[192];
nmpcVariables.x[202] += + nmpcWorkspace.evGx[1737]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1738]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1739]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1740]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1741]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1742]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1743]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1744]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1745]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[193];
nmpcVariables.x[203] += + nmpcWorkspace.evGx[1746]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1747]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1748]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1749]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1750]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1751]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1752]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1753]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1754]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[194];
nmpcVariables.x[204] += + nmpcWorkspace.evGx[1755]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1756]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1757]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1758]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1759]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1760]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1761]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1762]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1763]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[195];
nmpcVariables.x[205] += + nmpcWorkspace.evGx[1764]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1765]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1766]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1767]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1768]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1769]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1770]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1771]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1772]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[196];
nmpcVariables.x[206] += + nmpcWorkspace.evGx[1773]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1774]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1775]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1776]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1777]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1778]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1779]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1780]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1781]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[197];
nmpcVariables.x[207] += + nmpcWorkspace.evGx[1782]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1783]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1784]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1785]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1786]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1787]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1788]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1789]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1790]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[198];
nmpcVariables.x[208] += + nmpcWorkspace.evGx[1791]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1792]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1793]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1794]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1795]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1796]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1797]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1798]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1799]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[199];
nmpcVariables.x[209] += + nmpcWorkspace.evGx[1800]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1801]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1802]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1803]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1804]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1805]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1806]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1807]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1808]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[200];
nmpcVariables.x[210] += + nmpcWorkspace.evGx[1809]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1810]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1811]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1812]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1813]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1814]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1815]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1816]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1817]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[201];
nmpcVariables.x[211] += + nmpcWorkspace.evGx[1818]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1819]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1820]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1821]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1822]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1823]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1824]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1825]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1826]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[202];
nmpcVariables.x[212] += + nmpcWorkspace.evGx[1827]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1828]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1829]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1830]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1831]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1832]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1833]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1834]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1835]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[203];
nmpcVariables.x[213] += + nmpcWorkspace.evGx[1836]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1837]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1838]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1839]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1840]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1841]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1842]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1843]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1844]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[204];
nmpcVariables.x[214] += + nmpcWorkspace.evGx[1845]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1846]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1847]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1848]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1849]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1850]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1851]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1852]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1853]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[205];
nmpcVariables.x[215] += + nmpcWorkspace.evGx[1854]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1855]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1856]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1857]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1858]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1859]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1860]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1861]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1862]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[206];
nmpcVariables.x[216] += + nmpcWorkspace.evGx[1863]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1864]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1865]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1866]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1867]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1868]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1869]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1870]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1871]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[207];
nmpcVariables.x[217] += + nmpcWorkspace.evGx[1872]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1873]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1874]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1875]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1876]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1877]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1878]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1879]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1880]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[208];
nmpcVariables.x[218] += + nmpcWorkspace.evGx[1881]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1882]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1883]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1884]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1885]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1886]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1887]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1888]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1889]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[209];
nmpcVariables.x[219] += + nmpcWorkspace.evGx[1890]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1891]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1892]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1893]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1894]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1895]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1896]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1897]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1898]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[210];
nmpcVariables.x[220] += + nmpcWorkspace.evGx[1899]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1900]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1901]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1902]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1903]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1904]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1905]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1906]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1907]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[211];
nmpcVariables.x[221] += + nmpcWorkspace.evGx[1908]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1909]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1910]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1911]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1912]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1913]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1914]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1915]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1916]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[212];
nmpcVariables.x[222] += + nmpcWorkspace.evGx[1917]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1918]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1919]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1920]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1921]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1922]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1923]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1924]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1925]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[213];
nmpcVariables.x[223] += + nmpcWorkspace.evGx[1926]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1927]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1928]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1929]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1930]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1931]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1932]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1933]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1934]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[214];
nmpcVariables.x[224] += + nmpcWorkspace.evGx[1935]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1936]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1937]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1938]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1939]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1940]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1941]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1942]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1943]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[215];
nmpcVariables.x[225] += + nmpcWorkspace.evGx[1944]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1945]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1946]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1947]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1948]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1949]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1950]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1951]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1952]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[216];
nmpcVariables.x[226] += + nmpcWorkspace.evGx[1953]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1954]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1955]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1956]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1957]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1958]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1959]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1960]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1961]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[217];
nmpcVariables.x[227] += + nmpcWorkspace.evGx[1962]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1963]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1964]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1965]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1966]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1967]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1968]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1969]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1970]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[218];
nmpcVariables.x[228] += + nmpcWorkspace.evGx[1971]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1972]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1973]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1974]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1975]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1976]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1977]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1978]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1979]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[219];
nmpcVariables.x[229] += + nmpcWorkspace.evGx[1980]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1981]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1982]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1983]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1984]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1985]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1986]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1987]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1988]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[220];
nmpcVariables.x[230] += + nmpcWorkspace.evGx[1989]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1990]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[1991]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[1992]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[1993]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[1994]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[1995]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[1996]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[1997]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[221];
nmpcVariables.x[231] += + nmpcWorkspace.evGx[1998]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[1999]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2000]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2001]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2002]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2003]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2004]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2005]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2006]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[222];
nmpcVariables.x[232] += + nmpcWorkspace.evGx[2007]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2008]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2009]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2010]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2011]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2012]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2013]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2014]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2015]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[223];
nmpcVariables.x[233] += + nmpcWorkspace.evGx[2016]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2017]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2018]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2019]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2020]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2021]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2022]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2023]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2024]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[224];
nmpcVariables.x[234] += + nmpcWorkspace.evGx[2025]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2026]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2027]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2028]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2029]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2030]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2031]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2032]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2033]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[225];
nmpcVariables.x[235] += + nmpcWorkspace.evGx[2034]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2035]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2036]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2037]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2038]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2039]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2040]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2041]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2042]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[226];
nmpcVariables.x[236] += + nmpcWorkspace.evGx[2043]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2044]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2045]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2046]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2047]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2048]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2049]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2050]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2051]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[227];
nmpcVariables.x[237] += + nmpcWorkspace.evGx[2052]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2053]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2054]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2055]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2056]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2057]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2058]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2059]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2060]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[228];
nmpcVariables.x[238] += + nmpcWorkspace.evGx[2061]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2062]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2063]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2064]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2065]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2066]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2067]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2068]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2069]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[229];
nmpcVariables.x[239] += + nmpcWorkspace.evGx[2070]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2071]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2072]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2073]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2074]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2075]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2076]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2077]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2078]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[230];
nmpcVariables.x[240] += + nmpcWorkspace.evGx[2079]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2080]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2081]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2082]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2083]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2084]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2085]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2086]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2087]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[231];
nmpcVariables.x[241] += + nmpcWorkspace.evGx[2088]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2089]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2090]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2091]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2092]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2093]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2094]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2095]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2096]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[232];
nmpcVariables.x[242] += + nmpcWorkspace.evGx[2097]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2098]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2099]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2100]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2101]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2102]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2103]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2104]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2105]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[233];
nmpcVariables.x[243] += + nmpcWorkspace.evGx[2106]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2107]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2108]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2109]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2110]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2111]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2112]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2113]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2114]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[234];
nmpcVariables.x[244] += + nmpcWorkspace.evGx[2115]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2116]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2117]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2118]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2119]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2120]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2121]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2122]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2123]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[235];
nmpcVariables.x[245] += + nmpcWorkspace.evGx[2124]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2125]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2126]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2127]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2128]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2129]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2130]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2131]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2132]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[236];
nmpcVariables.x[246] += + nmpcWorkspace.evGx[2133]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2134]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2135]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2136]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2137]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2138]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2139]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2140]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2141]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[237];
nmpcVariables.x[247] += + nmpcWorkspace.evGx[2142]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2143]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2144]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2145]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2146]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2147]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2148]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2149]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2150]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[238];
nmpcVariables.x[248] += + nmpcWorkspace.evGx[2151]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2152]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2153]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2154]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2155]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2156]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2157]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2158]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2159]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[239];
nmpcVariables.x[249] += + nmpcWorkspace.evGx[2160]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2161]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2162]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2163]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2164]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2165]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2166]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2167]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2168]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[240];
nmpcVariables.x[250] += + nmpcWorkspace.evGx[2169]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2170]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2171]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2172]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2173]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2174]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2175]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2176]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2177]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[241];
nmpcVariables.x[251] += + nmpcWorkspace.evGx[2178]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2179]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2180]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2181]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2182]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2183]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2184]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2185]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2186]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[242];
nmpcVariables.x[252] += + nmpcWorkspace.evGx[2187]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2188]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2189]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2190]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2191]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2192]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2193]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2194]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2195]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[243];
nmpcVariables.x[253] += + nmpcWorkspace.evGx[2196]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2197]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2198]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2199]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2200]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2201]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2202]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2203]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2204]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[244];
nmpcVariables.x[254] += + nmpcWorkspace.evGx[2205]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2206]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2207]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2208]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2209]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2210]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2211]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2212]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2213]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[245];
nmpcVariables.x[255] += + nmpcWorkspace.evGx[2214]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2215]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2216]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2217]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2218]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2219]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2220]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2221]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2222]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[246];
nmpcVariables.x[256] += + nmpcWorkspace.evGx[2223]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2224]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2225]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2226]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2227]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2228]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2229]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2230]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2231]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[247];
nmpcVariables.x[257] += + nmpcWorkspace.evGx[2232]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2233]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2234]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2235]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2236]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2237]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2238]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2239]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2240]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[248];
nmpcVariables.x[258] += + nmpcWorkspace.evGx[2241]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2242]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2243]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2244]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2245]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2246]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2247]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2248]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2249]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[249];
nmpcVariables.x[259] += + nmpcWorkspace.evGx[2250]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2251]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2252]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2253]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2254]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2255]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2256]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2257]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2258]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[250];
nmpcVariables.x[260] += + nmpcWorkspace.evGx[2259]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2260]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2261]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2262]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2263]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2264]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2265]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2266]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2267]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[251];
nmpcVariables.x[261] += + nmpcWorkspace.evGx[2268]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2269]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2270]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2271]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2272]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2273]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2274]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2275]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2276]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[252];
nmpcVariables.x[262] += + nmpcWorkspace.evGx[2277]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2278]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2279]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2280]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2281]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2282]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2283]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2284]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2285]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[253];
nmpcVariables.x[263] += + nmpcWorkspace.evGx[2286]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2287]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2288]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2289]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2290]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2291]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2292]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2293]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2294]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[254];
nmpcVariables.x[264] += + nmpcWorkspace.evGx[2295]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2296]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2297]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2298]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2299]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2300]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2301]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2302]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2303]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[255];
nmpcVariables.x[265] += + nmpcWorkspace.evGx[2304]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2305]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2306]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2307]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2308]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2309]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2310]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2311]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2312]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[256];
nmpcVariables.x[266] += + nmpcWorkspace.evGx[2313]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2314]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2315]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2316]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2317]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2318]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2319]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2320]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2321]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[257];
nmpcVariables.x[267] += + nmpcWorkspace.evGx[2322]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2323]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2324]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2325]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2326]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2327]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2328]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2329]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2330]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[258];
nmpcVariables.x[268] += + nmpcWorkspace.evGx[2331]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2332]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2333]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2334]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2335]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2336]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2337]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2338]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2339]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[259];
nmpcVariables.x[269] += + nmpcWorkspace.evGx[2340]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2341]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2342]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2343]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2344]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2345]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2346]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2347]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2348]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[260];
nmpcVariables.x[270] += + nmpcWorkspace.evGx[2349]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2350]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2351]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2352]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2353]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2354]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2355]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2356]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2357]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[261];
nmpcVariables.x[271] += + nmpcWorkspace.evGx[2358]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2359]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2360]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2361]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2362]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2363]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2364]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2365]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2366]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[262];
nmpcVariables.x[272] += + nmpcWorkspace.evGx[2367]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2368]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2369]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2370]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2371]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2372]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2373]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2374]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2375]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[263];
nmpcVariables.x[273] += + nmpcWorkspace.evGx[2376]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2377]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2378]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2379]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2380]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2381]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2382]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2383]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2384]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[264];
nmpcVariables.x[274] += + nmpcWorkspace.evGx[2385]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2386]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2387]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2388]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2389]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2390]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2391]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2392]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2393]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[265];
nmpcVariables.x[275] += + nmpcWorkspace.evGx[2394]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2395]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2396]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2397]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2398]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2399]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2400]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2401]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2402]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[266];
nmpcVariables.x[276] += + nmpcWorkspace.evGx[2403]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2404]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2405]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2406]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2407]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2408]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2409]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2410]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2411]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[267];
nmpcVariables.x[277] += + nmpcWorkspace.evGx[2412]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2413]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2414]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2415]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2416]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2417]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2418]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2419]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2420]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[268];
nmpcVariables.x[278] += + nmpcWorkspace.evGx[2421]*nmpcWorkspace.Dx0[0] + nmpcWorkspace.evGx[2422]*nmpcWorkspace.Dx0[1] + nmpcWorkspace.evGx[2423]*nmpcWorkspace.Dx0[2] + nmpcWorkspace.evGx[2424]*nmpcWorkspace.Dx0[3] + nmpcWorkspace.evGx[2425]*nmpcWorkspace.Dx0[4] + nmpcWorkspace.evGx[2426]*nmpcWorkspace.Dx0[5] + nmpcWorkspace.evGx[2427]*nmpcWorkspace.Dx0[6] + nmpcWorkspace.evGx[2428]*nmpcWorkspace.Dx0[7] + nmpcWorkspace.evGx[2429]*nmpcWorkspace.Dx0[8] + nmpcWorkspace.d[269];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multEDu( &(nmpcWorkspace.E[ lRun3 * 36 ]), &(nmpcWorkspace.x[ lRun2 * 4 ]), &(nmpcVariables.x[ lRun1 * 9 + 9 ]) );
}
}
}

int nmpc_preparationStep(  )
{
int ret;

ret = nmpc_modelSimulation();
nmpc_evaluateObjective(  );
nmpc_condensePrep(  );
return ret;
}

int nmpc_feedbackStep(  )
{
int tmp;

nmpc_condenseFdb(  );

tmp = nmpc_solve( );

nmpc_expand(  );
return tmp;
}

int nmpc_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&nmpcWorkspace, 0, sizeof( nmpcWorkspace ));
return ret;
}

void nmpc_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcWorkspace.state[0] = nmpcVariables.x[index * 9];
nmpcWorkspace.state[1] = nmpcVariables.x[index * 9 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[index * 9 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[index * 9 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[index * 9 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[index * 9 + 5];
nmpcWorkspace.state[6] = nmpcVariables.x[index * 9 + 6];
nmpcWorkspace.state[7] = nmpcVariables.x[index * 9 + 7];
nmpcWorkspace.state[8] = nmpcVariables.x[index * 9 + 8];
nmpcWorkspace.state[126] = nmpcVariables.u[index * 4];
nmpcWorkspace.state[127] = nmpcVariables.u[index * 4 + 1];
nmpcWorkspace.state[128] = nmpcVariables.u[index * 4 + 2];
nmpcWorkspace.state[129] = nmpcVariables.u[index * 4 + 3];

nmpc_integrate(nmpcWorkspace.state, index == 0);

nmpcVariables.x[index * 9 + 9] = nmpcWorkspace.state[0];
nmpcVariables.x[index * 9 + 10] = nmpcWorkspace.state[1];
nmpcVariables.x[index * 9 + 11] = nmpcWorkspace.state[2];
nmpcVariables.x[index * 9 + 12] = nmpcWorkspace.state[3];
nmpcVariables.x[index * 9 + 13] = nmpcWorkspace.state[4];
nmpcVariables.x[index * 9 + 14] = nmpcWorkspace.state[5];
nmpcVariables.x[index * 9 + 15] = nmpcWorkspace.state[6];
nmpcVariables.x[index * 9 + 16] = nmpcWorkspace.state[7];
nmpcVariables.x[index * 9 + 17] = nmpcWorkspace.state[8];
}
}

void nmpc_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcVariables.x[index * 9] = nmpcVariables.x[index * 9 + 9];
nmpcVariables.x[index * 9 + 1] = nmpcVariables.x[index * 9 + 10];
nmpcVariables.x[index * 9 + 2] = nmpcVariables.x[index * 9 + 11];
nmpcVariables.x[index * 9 + 3] = nmpcVariables.x[index * 9 + 12];
nmpcVariables.x[index * 9 + 4] = nmpcVariables.x[index * 9 + 13];
nmpcVariables.x[index * 9 + 5] = nmpcVariables.x[index * 9 + 14];
nmpcVariables.x[index * 9 + 6] = nmpcVariables.x[index * 9 + 15];
nmpcVariables.x[index * 9 + 7] = nmpcVariables.x[index * 9 + 16];
nmpcVariables.x[index * 9 + 8] = nmpcVariables.x[index * 9 + 17];
}

if (strategy == 1 && xEnd != 0)
{
nmpcVariables.x[270] = xEnd[0];
nmpcVariables.x[271] = xEnd[1];
nmpcVariables.x[272] = xEnd[2];
nmpcVariables.x[273] = xEnd[3];
nmpcVariables.x[274] = xEnd[4];
nmpcVariables.x[275] = xEnd[5];
nmpcVariables.x[276] = xEnd[6];
nmpcVariables.x[277] = xEnd[7];
nmpcVariables.x[278] = xEnd[8];
}
else if (strategy == 2) 
{
nmpcWorkspace.state[0] = nmpcVariables.x[270];
nmpcWorkspace.state[1] = nmpcVariables.x[271];
nmpcWorkspace.state[2] = nmpcVariables.x[272];
nmpcWorkspace.state[3] = nmpcVariables.x[273];
nmpcWorkspace.state[4] = nmpcVariables.x[274];
nmpcWorkspace.state[5] = nmpcVariables.x[275];
nmpcWorkspace.state[6] = nmpcVariables.x[276];
nmpcWorkspace.state[7] = nmpcVariables.x[277];
nmpcWorkspace.state[8] = nmpcVariables.x[278];
if (uEnd != 0)
{
nmpcWorkspace.state[126] = uEnd[0];
nmpcWorkspace.state[127] = uEnd[1];
nmpcWorkspace.state[128] = uEnd[2];
nmpcWorkspace.state[129] = uEnd[3];
}
else
{
nmpcWorkspace.state[126] = nmpcVariables.u[116];
nmpcWorkspace.state[127] = nmpcVariables.u[117];
nmpcWorkspace.state[128] = nmpcVariables.u[118];
nmpcWorkspace.state[129] = nmpcVariables.u[119];
}

nmpc_integrate(nmpcWorkspace.state, 1);

nmpcVariables.x[270] = nmpcWorkspace.state[0];
nmpcVariables.x[271] = nmpcWorkspace.state[1];
nmpcVariables.x[272] = nmpcWorkspace.state[2];
nmpcVariables.x[273] = nmpcWorkspace.state[3];
nmpcVariables.x[274] = nmpcWorkspace.state[4];
nmpcVariables.x[275] = nmpcWorkspace.state[5];
nmpcVariables.x[276] = nmpcWorkspace.state[6];
nmpcVariables.x[277] = nmpcWorkspace.state[7];
nmpcVariables.x[278] = nmpcWorkspace.state[8];
}
}

void nmpc_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
nmpcVariables.u[index * 4] = nmpcVariables.u[index * 4 + 4];
nmpcVariables.u[index * 4 + 1] = nmpcVariables.u[index * 4 + 5];
nmpcVariables.u[index * 4 + 2] = nmpcVariables.u[index * 4 + 6];
nmpcVariables.u[index * 4 + 3] = nmpcVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
nmpcVariables.u[116] = uEnd[0];
nmpcVariables.u[117] = uEnd[1];
nmpcVariables.u[118] = uEnd[2];
nmpcVariables.u[119] = uEnd[3];
}
}

real_t nmpc_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + nmpcWorkspace.g[0]*nmpcWorkspace.x[0] + nmpcWorkspace.g[1]*nmpcWorkspace.x[1] + nmpcWorkspace.g[2]*nmpcWorkspace.x[2] + nmpcWorkspace.g[3]*nmpcWorkspace.x[3] + nmpcWorkspace.g[4]*nmpcWorkspace.x[4] + nmpcWorkspace.g[5]*nmpcWorkspace.x[5] + nmpcWorkspace.g[6]*nmpcWorkspace.x[6] + nmpcWorkspace.g[7]*nmpcWorkspace.x[7] + nmpcWorkspace.g[8]*nmpcWorkspace.x[8] + nmpcWorkspace.g[9]*nmpcWorkspace.x[9] + nmpcWorkspace.g[10]*nmpcWorkspace.x[10] + nmpcWorkspace.g[11]*nmpcWorkspace.x[11] + nmpcWorkspace.g[12]*nmpcWorkspace.x[12] + nmpcWorkspace.g[13]*nmpcWorkspace.x[13] + nmpcWorkspace.g[14]*nmpcWorkspace.x[14] + nmpcWorkspace.g[15]*nmpcWorkspace.x[15] + nmpcWorkspace.g[16]*nmpcWorkspace.x[16] + nmpcWorkspace.g[17]*nmpcWorkspace.x[17] + nmpcWorkspace.g[18]*nmpcWorkspace.x[18] + nmpcWorkspace.g[19]*nmpcWorkspace.x[19] + nmpcWorkspace.g[20]*nmpcWorkspace.x[20] + nmpcWorkspace.g[21]*nmpcWorkspace.x[21] + nmpcWorkspace.g[22]*nmpcWorkspace.x[22] + nmpcWorkspace.g[23]*nmpcWorkspace.x[23] + nmpcWorkspace.g[24]*nmpcWorkspace.x[24] + nmpcWorkspace.g[25]*nmpcWorkspace.x[25] + nmpcWorkspace.g[26]*nmpcWorkspace.x[26] + nmpcWorkspace.g[27]*nmpcWorkspace.x[27] + nmpcWorkspace.g[28]*nmpcWorkspace.x[28] + nmpcWorkspace.g[29]*nmpcWorkspace.x[29] + nmpcWorkspace.g[30]*nmpcWorkspace.x[30] + nmpcWorkspace.g[31]*nmpcWorkspace.x[31] + nmpcWorkspace.g[32]*nmpcWorkspace.x[32] + nmpcWorkspace.g[33]*nmpcWorkspace.x[33] + nmpcWorkspace.g[34]*nmpcWorkspace.x[34] + nmpcWorkspace.g[35]*nmpcWorkspace.x[35] + nmpcWorkspace.g[36]*nmpcWorkspace.x[36] + nmpcWorkspace.g[37]*nmpcWorkspace.x[37] + nmpcWorkspace.g[38]*nmpcWorkspace.x[38] + nmpcWorkspace.g[39]*nmpcWorkspace.x[39] + nmpcWorkspace.g[40]*nmpcWorkspace.x[40] + nmpcWorkspace.g[41]*nmpcWorkspace.x[41] + nmpcWorkspace.g[42]*nmpcWorkspace.x[42] + nmpcWorkspace.g[43]*nmpcWorkspace.x[43] + nmpcWorkspace.g[44]*nmpcWorkspace.x[44] + nmpcWorkspace.g[45]*nmpcWorkspace.x[45] + nmpcWorkspace.g[46]*nmpcWorkspace.x[46] + nmpcWorkspace.g[47]*nmpcWorkspace.x[47] + nmpcWorkspace.g[48]*nmpcWorkspace.x[48] + nmpcWorkspace.g[49]*nmpcWorkspace.x[49] + nmpcWorkspace.g[50]*nmpcWorkspace.x[50] + nmpcWorkspace.g[51]*nmpcWorkspace.x[51] + nmpcWorkspace.g[52]*nmpcWorkspace.x[52] + nmpcWorkspace.g[53]*nmpcWorkspace.x[53] + nmpcWorkspace.g[54]*nmpcWorkspace.x[54] + nmpcWorkspace.g[55]*nmpcWorkspace.x[55] + nmpcWorkspace.g[56]*nmpcWorkspace.x[56] + nmpcWorkspace.g[57]*nmpcWorkspace.x[57] + nmpcWorkspace.g[58]*nmpcWorkspace.x[58] + nmpcWorkspace.g[59]*nmpcWorkspace.x[59] + nmpcWorkspace.g[60]*nmpcWorkspace.x[60] + nmpcWorkspace.g[61]*nmpcWorkspace.x[61] + nmpcWorkspace.g[62]*nmpcWorkspace.x[62] + nmpcWorkspace.g[63]*nmpcWorkspace.x[63] + nmpcWorkspace.g[64]*nmpcWorkspace.x[64] + nmpcWorkspace.g[65]*nmpcWorkspace.x[65] + nmpcWorkspace.g[66]*nmpcWorkspace.x[66] + nmpcWorkspace.g[67]*nmpcWorkspace.x[67] + nmpcWorkspace.g[68]*nmpcWorkspace.x[68] + nmpcWorkspace.g[69]*nmpcWorkspace.x[69] + nmpcWorkspace.g[70]*nmpcWorkspace.x[70] + nmpcWorkspace.g[71]*nmpcWorkspace.x[71] + nmpcWorkspace.g[72]*nmpcWorkspace.x[72] + nmpcWorkspace.g[73]*nmpcWorkspace.x[73] + nmpcWorkspace.g[74]*nmpcWorkspace.x[74] + nmpcWorkspace.g[75]*nmpcWorkspace.x[75] + nmpcWorkspace.g[76]*nmpcWorkspace.x[76] + nmpcWorkspace.g[77]*nmpcWorkspace.x[77] + nmpcWorkspace.g[78]*nmpcWorkspace.x[78] + nmpcWorkspace.g[79]*nmpcWorkspace.x[79] + nmpcWorkspace.g[80]*nmpcWorkspace.x[80] + nmpcWorkspace.g[81]*nmpcWorkspace.x[81] + nmpcWorkspace.g[82]*nmpcWorkspace.x[82] + nmpcWorkspace.g[83]*nmpcWorkspace.x[83] + nmpcWorkspace.g[84]*nmpcWorkspace.x[84] + nmpcWorkspace.g[85]*nmpcWorkspace.x[85] + nmpcWorkspace.g[86]*nmpcWorkspace.x[86] + nmpcWorkspace.g[87]*nmpcWorkspace.x[87] + nmpcWorkspace.g[88]*nmpcWorkspace.x[88] + nmpcWorkspace.g[89]*nmpcWorkspace.x[89] + nmpcWorkspace.g[90]*nmpcWorkspace.x[90] + nmpcWorkspace.g[91]*nmpcWorkspace.x[91] + nmpcWorkspace.g[92]*nmpcWorkspace.x[92] + nmpcWorkspace.g[93]*nmpcWorkspace.x[93] + nmpcWorkspace.g[94]*nmpcWorkspace.x[94] + nmpcWorkspace.g[95]*nmpcWorkspace.x[95] + nmpcWorkspace.g[96]*nmpcWorkspace.x[96] + nmpcWorkspace.g[97]*nmpcWorkspace.x[97] + nmpcWorkspace.g[98]*nmpcWorkspace.x[98] + nmpcWorkspace.g[99]*nmpcWorkspace.x[99] + nmpcWorkspace.g[100]*nmpcWorkspace.x[100] + nmpcWorkspace.g[101]*nmpcWorkspace.x[101] + nmpcWorkspace.g[102]*nmpcWorkspace.x[102] + nmpcWorkspace.g[103]*nmpcWorkspace.x[103] + nmpcWorkspace.g[104]*nmpcWorkspace.x[104] + nmpcWorkspace.g[105]*nmpcWorkspace.x[105] + nmpcWorkspace.g[106]*nmpcWorkspace.x[106] + nmpcWorkspace.g[107]*nmpcWorkspace.x[107] + nmpcWorkspace.g[108]*nmpcWorkspace.x[108] + nmpcWorkspace.g[109]*nmpcWorkspace.x[109] + nmpcWorkspace.g[110]*nmpcWorkspace.x[110] + nmpcWorkspace.g[111]*nmpcWorkspace.x[111] + nmpcWorkspace.g[112]*nmpcWorkspace.x[112] + nmpcWorkspace.g[113]*nmpcWorkspace.x[113] + nmpcWorkspace.g[114]*nmpcWorkspace.x[114] + nmpcWorkspace.g[115]*nmpcWorkspace.x[115] + nmpcWorkspace.g[116]*nmpcWorkspace.x[116] + nmpcWorkspace.g[117]*nmpcWorkspace.x[117] + nmpcWorkspace.g[118]*nmpcWorkspace.x[118] + nmpcWorkspace.g[119]*nmpcWorkspace.x[119];
kkt = fabs( kkt );
for (index = 0; index < 120; ++index)
{
prd = nmpcWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(nmpcWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(nmpcWorkspace.ub[index] * prd);
}
return kkt;
}

real_t nmpc_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 13 */
real_t tmpDy[ 13 ];

/** Row vector of size: 9 */
real_t tmpDyN[ 9 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[lRun1 * 9];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[lRun1 * 9 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[lRun1 * 9 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[lRun1 * 9 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[lRun1 * 9 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[lRun1 * 9 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[lRun1 * 9 + 6];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[lRun1 * 9 + 7];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[lRun1 * 9 + 8];
nmpcWorkspace.objValueIn[9] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.objValueIn[10] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.objValueIn[11] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.objValueIn[12] = nmpcVariables.u[lRun1 * 4 + 3];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[lRun1 * 13] = nmpcWorkspace.objValueOut[0] - nmpcVariables.y[lRun1 * 13];
nmpcWorkspace.Dy[lRun1 * 13 + 1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.y[lRun1 * 13 + 1];
nmpcWorkspace.Dy[lRun1 * 13 + 2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.y[lRun1 * 13 + 2];
nmpcWorkspace.Dy[lRun1 * 13 + 3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.y[lRun1 * 13 + 3];
nmpcWorkspace.Dy[lRun1 * 13 + 4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.y[lRun1 * 13 + 4];
nmpcWorkspace.Dy[lRun1 * 13 + 5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.y[lRun1 * 13 + 5];
nmpcWorkspace.Dy[lRun1 * 13 + 6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.y[lRun1 * 13 + 6];
nmpcWorkspace.Dy[lRun1 * 13 + 7] = nmpcWorkspace.objValueOut[7] - nmpcVariables.y[lRun1 * 13 + 7];
nmpcWorkspace.Dy[lRun1 * 13 + 8] = nmpcWorkspace.objValueOut[8] - nmpcVariables.y[lRun1 * 13 + 8];
nmpcWorkspace.Dy[lRun1 * 13 + 9] = nmpcWorkspace.objValueOut[9] - nmpcVariables.y[lRun1 * 13 + 9];
nmpcWorkspace.Dy[lRun1 * 13 + 10] = nmpcWorkspace.objValueOut[10] - nmpcVariables.y[lRun1 * 13 + 10];
nmpcWorkspace.Dy[lRun1 * 13 + 11] = nmpcWorkspace.objValueOut[11] - nmpcVariables.y[lRun1 * 13 + 11];
nmpcWorkspace.Dy[lRun1 * 13 + 12] = nmpcWorkspace.objValueOut[12] - nmpcVariables.y[lRun1 * 13 + 12];
}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[270];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[271];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[272];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[273];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[274];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[275];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[276];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[277];
nmpcWorkspace.objValueIn[8] = nmpcVariables.x[278];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0] - nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.yN[5];
nmpcWorkspace.DyN[6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.yN[6];
nmpcWorkspace.DyN[7] = nmpcWorkspace.objValueOut[7] - nmpcVariables.yN[7];
nmpcWorkspace.DyN[8] = nmpcWorkspace.objValueOut[8] - nmpcVariables.yN[8];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 13] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 26] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 39] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 52] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 65] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 78] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 91] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 104] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 117] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 130] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 143] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 156];
tmpDy[1] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169 + 1] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 14] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 27] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 40] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 53] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 66] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 79] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 92] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 105] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 118] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 131] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 144] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 157];
tmpDy[2] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169 + 2] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 15] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 28] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 41] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 54] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 67] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 80] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 93] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 106] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 119] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 132] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 145] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 158];
tmpDy[3] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169 + 3] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 16] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 29] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 42] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 55] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 68] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 81] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 94] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 107] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 120] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 133] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 146] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 159];
tmpDy[4] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169 + 4] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 17] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 30] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 43] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 56] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 69] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 82] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 95] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 108] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 121] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 134] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 147] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 160];
tmpDy[5] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169 + 5] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 18] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 31] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 44] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 57] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 70] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 83] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 96] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 109] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 122] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 135] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 148] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 161];
tmpDy[6] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169 + 6] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 19] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 32] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 45] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 58] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 71] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 84] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 97] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 110] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 123] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 136] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 149] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 162];
tmpDy[7] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169 + 7] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 20] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 33] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 46] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 59] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 72] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 85] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 98] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 111] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 124] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 137] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 150] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 163];
tmpDy[8] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169 + 8] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 21] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 34] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 47] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 60] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 73] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 86] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 99] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 112] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 125] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 138] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 151] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 164];
tmpDy[9] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169 + 9] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 22] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 35] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 48] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 61] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 74] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 87] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 100] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 113] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 126] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 139] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 152] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 165];
tmpDy[10] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169 + 10] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 23] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 36] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 49] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 62] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 75] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 88] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 101] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 114] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 127] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 140] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 153] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 166];
tmpDy[11] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169 + 11] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 24] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 37] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 50] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 63] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 76] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 89] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 102] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 115] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 128] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 141] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 154] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 167];
tmpDy[12] = + nmpcWorkspace.Dy[lRun1 * 13]*nmpcVariables.W[lRun1 * 169 + 12] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*nmpcVariables.W[lRun1 * 169 + 25] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*nmpcVariables.W[lRun1 * 169 + 38] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*nmpcVariables.W[lRun1 * 169 + 51] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*nmpcVariables.W[lRun1 * 169 + 64] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*nmpcVariables.W[lRun1 * 169 + 77] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*nmpcVariables.W[lRun1 * 169 + 90] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*nmpcVariables.W[lRun1 * 169 + 103] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*nmpcVariables.W[lRun1 * 169 + 116] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*nmpcVariables.W[lRun1 * 169 + 129] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*nmpcVariables.W[lRun1 * 169 + 142] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*nmpcVariables.W[lRun1 * 169 + 155] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*nmpcVariables.W[lRun1 * 169 + 168];
objVal += + nmpcWorkspace.Dy[lRun1 * 13]*tmpDy[0] + nmpcWorkspace.Dy[lRun1 * 13 + 1]*tmpDy[1] + nmpcWorkspace.Dy[lRun1 * 13 + 2]*tmpDy[2] + nmpcWorkspace.Dy[lRun1 * 13 + 3]*tmpDy[3] + nmpcWorkspace.Dy[lRun1 * 13 + 4]*tmpDy[4] + nmpcWorkspace.Dy[lRun1 * 13 + 5]*tmpDy[5] + nmpcWorkspace.Dy[lRun1 * 13 + 6]*tmpDy[6] + nmpcWorkspace.Dy[lRun1 * 13 + 7]*tmpDy[7] + nmpcWorkspace.Dy[lRun1 * 13 + 8]*tmpDy[8] + nmpcWorkspace.Dy[lRun1 * 13 + 9]*tmpDy[9] + nmpcWorkspace.Dy[lRun1 * 13 + 10]*tmpDy[10] + nmpcWorkspace.Dy[lRun1 * 13 + 11]*tmpDy[11] + nmpcWorkspace.Dy[lRun1 * 13 + 12]*tmpDy[12];
}

tmpDyN[0] = + nmpcWorkspace.DyN[0]*nmpcVariables.WN[0];
tmpDyN[1] = + nmpcWorkspace.DyN[1]*nmpcVariables.WN[10];
tmpDyN[2] = + nmpcWorkspace.DyN[2]*nmpcVariables.WN[20];
tmpDyN[3] = + nmpcWorkspace.DyN[3]*nmpcVariables.WN[30];
tmpDyN[4] = + nmpcWorkspace.DyN[4]*nmpcVariables.WN[40];
tmpDyN[5] = + nmpcWorkspace.DyN[5]*nmpcVariables.WN[50];
tmpDyN[6] = + nmpcWorkspace.DyN[6]*nmpcVariables.WN[60];
tmpDyN[7] = + nmpcWorkspace.DyN[7]*nmpcVariables.WN[70];
tmpDyN[8] = + nmpcWorkspace.DyN[8]*nmpcVariables.WN[80];
objVal += + nmpcWorkspace.DyN[0]*tmpDyN[0] + nmpcWorkspace.DyN[1]*tmpDyN[1] + nmpcWorkspace.DyN[2]*tmpDyN[2] + nmpcWorkspace.DyN[3]*tmpDyN[3] + nmpcWorkspace.DyN[4]*tmpDyN[4] + nmpcWorkspace.DyN[5]*tmpDyN[5] + nmpcWorkspace.DyN[6]*tmpDyN[6] + nmpcWorkspace.DyN[7]*tmpDyN[7] + nmpcWorkspace.DyN[8]*tmpDyN[8];

objVal *= 0.5;
return objVal;
}

