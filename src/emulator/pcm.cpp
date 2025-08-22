/*
 * Copyright (C) 2021, 2024 nukeykt
 *
 *  Redistribution and use of this code or any derivative works are permitted
 *  provided that the following conditions are met:
 *
 *   - Redistributions may not be sold, nor may they be used in a commercial
 *     product or activity.
 *
 *   - Redistributions that are modified from the original source must include the
 *     complete source code, including the source code for all components used by a
 *     binary built from the modified sources. However, as a special exception, the
 *     source code distributed need not include anything that is normally distributed
 *     (in either source or binary form) with the major components (compiler, kernel,
 *     and so on) of the operating system on which the executable runs, unless that
 *     component itself accompanies the executable.
 *
 *   - Redistributions must reproduce the above copyright notice, this list of
 *     conditions and the following disclaimer in the documentation and/or other
 *     materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mcu.h"
#include "pcm.h"

Pcm::Pcm(MCU *mcu): mcu(mcu) {}

void Pcm::PCM_Write(uint32_t address, uint8_t data)
{
    address &= 0x3f;
    if (address < 0x4) // voice enable
    {
        switch (address & 3)
        {
            case 0:
                pcm.voice_mask_pending &= ~0xf000000;
                pcm.voice_mask_pending |= (data & 0xf) << 24;
                break;
            case 1:
                pcm.voice_mask_pending &= ~0xff0000;
                pcm.voice_mask_pending |= (data & 0xff) << 16;
                break;
            case 2:
                pcm.voice_mask_pending &= ~0xff00;
                pcm.voice_mask_pending |= (data & 0xff) << 8;
                break;
            case 3:
                pcm.voice_mask_pending &= ~0xff;
                pcm.voice_mask_pending |= (data & 0xff) << 0;
                break;
        }
        pcm.voice_mask_updating = 1;
    }
    else if (address >= 0x20 && address < 0x24) // wave rom
    {
        switch (address & 3)
        {
            case 1:
                pcm.wave_read_address &= ~0xff0000;
                pcm.wave_read_address |= (data & 0xff) << 16;
                break;
            case 2:
                pcm.wave_read_address &= ~0xff00;
                pcm.wave_read_address |= (data & 0xff) << 8;
                break;
            case 3:
                pcm.wave_read_address &= ~0xff;
                pcm.wave_read_address |= (data & 0xff) << 0;
                pcm.wave_byte_latch = PCM_ReadROM(pcm.wave_read_address);
                break;
        }
    }
    else if (address == 0x3c)
    {
        pcm.config_reg_3c = data;
    }
    else if (address == 0x3d)
    {
        pcm.config_reg_3d = data;
    }
    else if (address == 0x3e)
    {
        pcm.select_channel = data & 0x1f;
    }
    else if ((address >= 0x4 && address < 0x10) || (address >= 0x24 && address < 0x30))
    {
        switch (address & 3)
        {
            case 1:
                pcm.write_latch &= ~0xf0000;
                pcm.write_latch |= (data & 0xf) << 16;
                break;
            case 2:
                pcm.write_latch &= ~0xff00;
                pcm.write_latch |= (data & 0xff) << 8;
                break;
            case 3:
                pcm.write_latch &= ~0xff;
                pcm.write_latch |= (data & 0xff) << 0;
                break;
        }
        if ((address & 3) == 3)
        {
            int ix = 0;
            if (address & 32)
                ix |= 1;
            if ((address & 8) == 0)
                ix |= 4;
            if ((address & 4) == 0)
                ix |= 2;

             pcm_lock.Acquire();
            pcm.ram1[pcm.select_channel][ix] = pcm.write_latch;
             pcm_lock.Release();
        }
    }
    else if ((address >= 0x10 && address < 0x20) || (address >= 0x30 && address < 0x38))
    {
        switch (address & 1)
        {
        case 0:
            pcm.write_latch &= ~0xff00;
            pcm.write_latch |= (data & 0xff) << 8;
            break;
        case 1:
            pcm.write_latch &= ~0xff;
            pcm.write_latch |= (data & 0xff) << 0;
            break;
        }
        if ((address & 1) == 1)
        {
            int ix = (address >> 1) & 7;
            if (address & 32)
                ix |= 8;

             pcm_lock.Acquire();
            pcm.ram2[pcm.select_channel][ix] = pcm.write_latch;
             pcm_lock.Release();
        }
    }
}

// rv: [30][2], [30][3]
// ch: [31][2], [31][5]

uint8_t Pcm::PCM_Read(uint32_t address)
{
    address &= 0x3f;
    //printf("PCM Read: %.2x\n", address);

    if (address < 0x4)
    {
        if (pcm.voice_mask_updating)
            pcm.voice_mask = pcm.voice_mask_pending;
        pcm.voice_mask_updating = 0;
    }
    else if (address == 0x3c || address == 0x3e) // status
    {
        pcm_lock.Acquire();
        uint8_t status = 0;
        if (address == 0x3e && pcm.irq_assert)
        {
            pcm.irq_assert = 0;
            mcu->MCU_GA_SetGAInt(5, 0);
        }

        status |= pcm.irq_channel;
        if (pcm.voice_mask_updating)
            status |= 32;
         pcm_lock.Release();

        return status;
    }
    else if (address == 0x3f)
    {
        return pcm.wave_byte_latch;
    }
    else if ((address >= 0x4 && address < 0x10) || (address >= 0x24 && address < 0x30))
    {
        if ((address & 3) == 1)
        {
            int ix = 0;
            if (address & 32)
                ix |= 1;
            if ((address & 8) == 0)
                ix |= 4;
            if ((address & 4) == 0)
                ix |= 2;

            pcm.read_latch = pcm.ram1[pcm.select_channel][ix];
        }
    }
    else if ((address >= 0x10 && address < 0x20) || (address >= 0x30 && address < 0x38))
    {
        if ((address & 1) == 0)
        {
            int ix = (address >> 1) & 7;
            if (address & 32)
                ix |= 8;

            pcm.read_latch = pcm.ram2[pcm.select_channel][ix];
        }
    }
    else if (address >= 0x39 && address <= 0x3b)
    {
        switch (address & 3)
        {
            case 1:
                return (pcm.read_latch >> 16) & 0xf;
            case 2:
                return (pcm.read_latch >> 8) & 0xff;
            case 3:
                return (pcm.read_latch >> 0) & 0xff;
        }
    }

    return 0;
}

void Pcm::PCM_Reset(void)
{
    memset(&pcm, 0, sizeof(pcm));
}

// Sign-extends a 20-bit signed integer to a 32-bit signed integer.
constexpr inline int32_t sx20(int32_t in)
{
    return (in << 12) >> 12;
}

inline int32_t addclip20(int32_t add1, int32_t add2, int32_t cin)
{
    return sx20(add1) + sx20(add2) + cin;
}

inline int32_t multi(int32_t val1, int8_t val2)
{
    return sx20(val1) * val2;
    // return (val1) * val2;
}

static const int interp_lut[3][128] = {
    3385, 3401, 3417, 3432, 3448, 3463, 3478, 3492, 3506, 3521, 3534, 3548, 3562, 3575, 3588, 3601,
    3614, 3626, 3638, 3650, 3662, 3673, 3685, 3696, 3707, 3718, 3728, 3739, 3749, 3759, 3768, 3778,
    3787, 3796, 3805, 3814, 3823, 3831, 3839, 3847, 3855, 3863, 3870, 3878, 3885, 3892, 3899, 3905,
    3912, 3918, 3924, 3930, 3936, 3942, 3948, 3953, 3958, 3963, 3968, 3973, 3978, 3983, 3987, 3991,
    3995, 4000, 4004, 4007, 4011, 4015, 4018, 4022, 4025, 4028, 4031, 4034, 4037, 4040, 4042, 4045,
    4047, 4050, 4052, 4054, 4057, 4059, 4061, 4063, 4064, 4066, 4068, 4070, 4071, 4073, 4074, 4076,
    4077, 4078, 4079, 4081, 4082, 4083, 4084, 4085, 4086, 4086, 4087, 4088, 4089, 4089, 4090, 4091,
    4091, 4092, 4092, 4093, 4093, 4094, 4094, 4094, 4094, 4095, 4095, 4095, 4095, 4095, 4095, 4095,

    710, 726, 742, 758, 775, 792, 809, 826, 844, 861, 879, 897, 915, 933, 952, 971,
    990, 1009, 1028, 1047, 1067, 1087, 1106, 1126, 1147, 1167, 1188, 1208, 1229, 1250, 1271, 1292,
    1314, 1335, 1357, 1379, 1400, 1423, 1445, 1467, 1489, 1512, 1534, 1557, 1580, 1602, 1625, 1648,
    1671, 1695, 1718, 1741, 1764, 1788, 1811, 1835, 1858, 1882, 1906, 1929, 1953, 1977, 2000, 2024,
    2048, 2069, 2095, 2119, 2143, 2166, 2190, 2214, 2237, 2261, 2284, 2308, 2331, 2355, 2378, 2401,
    2425, 2448, 2471, 2494, 2517, 2539, 2562, 2585, 2607, 2630, 2652, 2674, 2696, 2718, 2740, 2762,
    2783, 2805, 2826, 2847, 2868, 2889, 2910, 2931, 2951, 2971, 2991, 3011, 3031, 3051, 3070, 3089,
    3108, 3127, 3146, 3164, 3182, 3200, 3218, 3236, 3253, 3271, 3288, 3304, 3321, 3338, 3354, 3370,

    0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6,
    6, 7, 8, 8, 9, 10, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 22, 23, 24, 26, 27, 29, 30, 32, 34, 36, 38, 40, 42, 44, 46,
    49, 51, 53, 56, 59, 62, 65, 68, 71, 74, 77, 81, 84, 88, 92, 96,
    100, 104, 109, 113, 118, 122, 127, 132, 137, 143, 148, 154, 160, 165, 171, 178,
    184, 191, 197, 204, 211, 219, 226, 234, 241, 249, 257, 266, 274, 283, 292, 301,
    310, 319, 329, 339, 349, 359, 369, 380, 391, 402, 413, 424, 436, 448, 460, 472,
    484, 497, 510, 523, 536, 549, 563, 577, 591, 605, 619, 634, 648, 663, 679, 694,
};

static uint8_t flip_nibble_lut[16] = {0,8,4,12,2,10,6,14,1,9,5,13,3,11,7,15};
static uint8_t tvc_lut[4] = {3,15,63,127};

inline void calc_tv(pcm_t *pcm, int e, int adjust, uint16_t *levelcur, int active, int *volmul)
{
    *levelcur &= 0x7fff;

    uint8_t speed = adjust & 0xff;
    uint8_t speed_and_0x20 = speed & 0x20;
    uint8_t speed_and_0x40 = speed & 0x40;
    uint8_t speed_and_0x80 = speed & 0x80;
    uint16_t target = (adjust >> 8) & 0xff;

    uint8_t w1 = !(speed & 0xf0);
    uint8_t w2 = w1 || (speed & 0x10);
    uint8_t w3 = (!speed_and_0x80 || (!speed_and_0x40 && (!w2 || !speed_and_0x20)));

    uint8_t type = w2 | (w3 << 3);
    if (speed_and_0x20)
        type |= 2;
    if (!speed_and_0x80 || !speed_and_0x40)
        type |= 4;

    uint8_t write = !active;
    uint8_t addlow = 0;
    if (type & 4) {
        addlow = flip_nibble_lut[pcm->tv_counter & 0x0f];
        write |= 1;
    } else {
        uint8_t t = type & 3;
        addlow = flip_nibble_lut[(pcm->tv_counter >> (2 * t + 2)) & 0x0f];
        write |= !(pcm->tv_counter & tvc_lut[t]);
    }

    if ((type & 8) == 0)
    {
        int shift = speed & 15;
        shift = (10 - shift) & 15;

        int sum1 = (target << 11); // 5
        if (e != 2 || active)
            sum1 -= (*levelcur << 4); // 6

        int preshift = sum1;

        int shifted = preshift >> shift;
        shifted -= sum1;

        int sum2 = (target << 11) + addlow + shifted;
        if (write)
            *levelcur = (sum2 >> 4) & 0x7fff;

        if (e < 2)
            *volmul = (sum2 >> 4) & 0x7ffe;
    }
    else
    {
        int shift = (speed >> 4) & 14;
        shift |= w2;
        shift = (10 - shift) & 15;

        int sum1 = target << 11; // 5
        if (e != 2 || active)
            sum1 -= (*levelcur << 4); // 6
        int neg = (sum1 & 0x80000) != 0;
        int preshift = (speed & 15) << 9;
        if (!w1)
            preshift |= 0x2000;
        if (neg)
            preshift ^= ~0x3f;

        int shifted = preshift >> shift;
        int sum2 = shifted;
        if (e != 2 || active)
            sum2 += (*levelcur << 4) | addlow;

        int sum2_l = (sum2 >> 4);

        int sum3 = (target << 11) - (sum2_l << 4);

        int neg2 = (sum3 & 0x80000) != 0;
        int xnor = !(neg2 ^ neg);

        if (write)
        {
            if (xnor)
                *levelcur = sum2_l & 0x7fff;
            else
                *levelcur = target << 7;
        }

        if (e == 0)
        {
            *volmul = sum2_l & 0x7ffe;
        }
        else if (e == 1)
        {
            if (xnor)
                *volmul = sum2_l & 0x7ffe;
            else
                *volmul = target << 7;
        }
    }
}

inline int eram_unpack(pcm_t *pcm, int addr, int type = 0)
{
    addr &= 0x3fff;
    int data = pcm->eram[addr];
    int val = data & 0x3fff;
    int sh = (data >> 14) & 3;

    val <<= 18;
    return val >> (18 - sh * 2 + type);
}

inline void eram_pack(pcm_t *pcm, int addr, int val)
{
    addr &= 0x3fff;
    int sh = 0;
    int top = (val >> 13) & 0x7f;
    if (top & 0x40)
        top ^= 0x7f;
    if (top >= 16)
        sh = 3;
    else if (top >= 4)
        sh = 2;
    else if (top >= 1)
        sh = 1;
    else
        sh = 0;

    int data = (val >> (sh * 2)) & 0x3fff;
    data |= sh << 14;
    pcm->eram[addr] = data;
}

/*
void Pcm::PCM_Update(uint64_t cycles)
{
    // int reg_slots = (pcm.config_reg_3d & 31) + 1;
    constexpr int reg_slots = 28;
    int voice_active = pcm.voice_mask & pcm.voice_mask_pending;
    while (pcm.cycles < cycles)
    {
         pcm_lock.Acquire();

        int tt[2] = {};

        { // final mixing
            int noise_mask = 0;
            int orval = 0;
            int write_mask = 3;
            // int dac_mask = -4;


            int shifter = pcm.ram2[30][10];
            int xr = ((shifter >> 0) ^ (shifter >> 1) ^ (shifter >> 7) ^ (shifter >> 12)) & 1;
            shifter = (shifter >> 1) | (xr << 15);
            pcm.ram2[30][10] = shifter;

            pcm.accum_l = addclip20(pcm.accum_l, pcm.ram1[30][0], 0);
            pcm.accum_r = addclip20(pcm.accum_r, pcm.ram1[30][1], 0);

            pcm.ram1[30][2] = addclip20(pcm.accum_l,
                orval | (shifter & noise_mask), 0);

            pcm.ram1[30][4] = addclip20(pcm.accum_r,
                orval | (shifter & noise_mask), 0);

            pcm.ram1[30][0] = pcm.accum_l & write_mask;
            pcm.ram1[30][1] = pcm.accum_r & write_mask;
            

            tt[0] = (int)((pcm.ram1[30][2] & ~write_mask) << 12);
            tt[1] = (int)((pcm.ram1[30][4] & ~write_mask) << 12);

            mcu->MCU_PostSample(tt);

            xr = ((shifter >> 0) ^ (shifter >> 1) ^ (shifter >> 7) ^ (shifter >> 12)) & 1;
            shifter = (shifter >> 1) | (xr << 15);

            pcm.accum_l = addclip20(pcm.accum_l, pcm.ram1[30][0], 0);
            pcm.accum_r = addclip20(pcm.accum_r, pcm.ram1[30][1], 0);

            pcm.ram1[30][3] = addclip20(pcm.accum_l,
                orval | (shifter & noise_mask), 0);

            pcm.ram1[30][5] = addclip20(pcm.accum_r,
                orval | (shifter & noise_mask), 0);

            // if (pcm.config_reg_3c & 0x40) // oversampling
            // if (true) // oversampling
            if (false) // oversampling
            {
                pcm.ram2[30][10] = shifter;

                pcm.ram1[30][0] = pcm.accum_l & write_mask;
                pcm.ram1[30][1] = pcm.accum_r & write_mask;


                tt[0] = (int)((pcm.ram1[30][3] & ~write_mask) << 12);
                tt[1] = (int)((pcm.ram1[30][5] & ~write_mask) << 12);

                mcu->MCU_PostSample(tt);
            }
        }

        { // global counter for envelopes
            if (!pcm.nfs)
                pcm.tv_counter = pcm.ram2[31][8]; // fixme

            pcm.tv_counter -= 1;

            pcm.tv_counter &= 0x3fff;
        }

        // chorus/reverb

        { // fixme
            if (pcm.ram2[31][8] & 0x8000)
                pcm.ram2[31][9] = pcm.ram2[31][8] & 0x7fff;
            else
                pcm.ram2[31][10] = pcm.ram2[31][8] & 0x7fff;

            if ((0x4000 - pcm.ram2[31][8]) & 0x8000)
                pcm.ram2[31][10] = (0x4000 - pcm.ram2[31][8]) & 0x7fff;
            else
                pcm.ram2[31][9] = (0x4000 - pcm.ram2[31][8]) & 0x7fff;
        }

        {
            int v1 = pcm.ram2[31][1];

            int m1 = multi(pcm.ram1[29][1], v1 >> 8) >> 5; // 14
            int m2 = multi(pcm.rcsum[1], v1 & 255) >> 5; // 15

            pcm.ram1[29][1] = addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1); // 16
        }

        {
            int okey = (pcm.ram2[31][7] & 0x20) != 0;
            int key = 1;
            int active = okey && key;
            int u = 0;
            calc_tv(&pcm, 1, pcm.ram2[30][0], &pcm.ram2[30][9], active, &u);
        }

        {
            int v1 = pcm.ram2[30][1];
            int m1 = multi(pcm.ram1[29][0], v1 >> 8) >> 5; // 17
            int m2 = multi(pcm.rcsum[0], v1 & 255) >> 5; // 18

            pcm.ram1[29][0] = addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1); // 19
        }

        int rcadd[6] = {};
        int rcadd2[6] = {};

        {
            {
                // 1
                int v1 = pcm.ram2[30][4];
                int m1 = multi(pcm.ram1[29][0], (v1 >> 8)) >> 6;
                int v2 = 0;
                int s1 = eram_unpack(&pcm, pcm.ram2[28][1] + pcm.tv_counter, 1);
                int s2 = eram_unpack(&pcm, pcm.ram2[28][1] + pcm.tv_counter);
                if ((v1 & 0x30) != 0)
                {
                    v2 = s1;
                }
                int v3 = addclip20(m1, v2 ^ 0xfffff, 1);
                pcm.ram1[29][4] = v3;
                int m2 = multi(v3, v1 & 255) >> 5;
                pcm.ram1[29][5] = addclip20(m2 >> 1, s2, m2 & 1);
            }
            {
                // 2
                int v1 = pcm.ram2[30][4];
                int v2 = 0;
                int s1 = eram_unpack(&pcm, pcm.ram2[28][2] + pcm.tv_counter, 1);
                int s2 = eram_unpack(&pcm, pcm.ram2[28][2] + pcm.tv_counter);
                if ((v1 & 0x30) != 0)
                {
                    v2 = s1;
                }
                int v3 = addclip20(pcm.ram1[29][5], v2 ^ 0xfffff, 1);
                pcm.ram1[29][5] = v3;
                int m2 = multi(v3, v1 & 255) >> 5;
                pcm.ram1[28][0] = addclip20(m2 >> 1, s2, m2 & 1);
            }
            {
                // 3
                int v1 = pcm.ram2[30][4];
                int v2 = 0;
                int s1 = eram_unpack(&pcm, pcm.ram2[28][3] + pcm.tv_counter, 1);
                int s2 = eram_unpack(&pcm, pcm.ram2[28][3] + pcm.tv_counter);
                if ((v1 & 0x30) != 0)
                {
                    v2 = s1;
                }
                int v3 = addclip20(pcm.ram1[28][0], v2 ^ 0xfffff, 1);
                pcm.ram1[28][0] = v3;
                int m2 = multi(v3, v1 & 255) >> 5;
                pcm.ram1[28][1] = addclip20(m2 >> 1, s2, m2 & 1);


                pcm.ram1[28][2] = eram_unpack(&pcm, pcm.ram2[28][5] + pcm.tv_counter);
            }
            {
                // 4
                int v1 = pcm.ram2[30][5];
                int v2 = 0;
                int s1 = eram_unpack(&pcm, pcm.ram2[28][4] + pcm.tv_counter, 1);
                int s2 = eram_unpack(&pcm, pcm.ram2[28][4] + pcm.tv_counter);
                if ((v1 & 0x30) != 0)
                {
                    v2 = s1;
                }
                int v3 = addclip20(pcm.ram1[28][1], v2 ^ 0xfffff, 1);
                pcm.ram1[28][1] = v3;
                int m2 = multi(v3, v1 & 255) >> 5;
                pcm.ram1[28][3] = addclip20(m2 >> 1, s2, m2 & 1);


                pcm.ram1[28][4] = eram_unpack(&pcm, pcm.ram2[29][1] + pcm.tv_counter);
            }
            {
                // 5

                int v1 = pcm.ram2[30][7];
                int m1 = multi(pcm.ram1[29][2], (v1 >> 8)) >> 5;
                int s1 = eram_unpack(&pcm, pcm.ram2[29][0] + pcm.tv_counter);
                int m2 = multi(s1, v1 & 255) >> 5;
                pcm.ram1[29][2] = addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1);

                eram_pack(&pcm, pcm.ram2[28][0] + pcm.tv_counter, pcm.ram1[29][4]);
            }
            {
                // 6

                int v1 = pcm.ram2[30][8];
                int m1 = multi(pcm.ram1[29][3], (v1 >> 8)) >> 5;
                int s1 = eram_unpack(&pcm, pcm.ram2[29][8] + pcm.tv_counter);
                int m2 = multi(s1, v1 & 255) >> 5;
                pcm.ram1[29][3] = addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1);

                eram_pack(&pcm, pcm.ram2[28][1] + pcm.tv_counter, pcm.ram1[29][5]);

                eram_pack(&pcm, pcm.ram2[28][2] + pcm.tv_counter, pcm.ram1[28][0]);
            }
            {
                // 7

                int v1 = pcm.ram2[30][9];
                int v2 = pcm.ram1[28][3];
                int m1 = multi(pcm.ram1[29][2], (v1 >> 8)) >> 5;
                int m2 = multi(pcm.ram1[29][3], (v1 >> 8)) >> 5;
                pcm.ram1[28][3] = addclip20(v2, m1 >> 1, m1 & 1);
                pcm.ram1[28][5] = addclip20(v2, m2 >> 1, m2 & 1);

                eram_pack(&pcm, pcm.ram2[28][3] + pcm.tv_counter, pcm.ram1[28][1]);
            }
            {
                // 8

                int v1 = pcm.ram2[30][6];
                int m1 = multi(pcm.ram1[28][2], v1 >> 8) >> 5;

                int v2 = addclip20(pcm.ram1[28][3], m1 >> 1, m1 & 1);
                pcm.ram1[28][3] = v2;
                int m2 = multi(v2, v1 & 255) >> 5;
                pcm.ram1[28][2] = addclip20(pcm.ram1[28][2], m2 >> 1, m2 & 1);


                pcm.ram1[28][1] = eram_unpack(&pcm, pcm.ram2[28][9] + pcm.tv_counter);
            }
            {
                // 9

                int v1 = pcm.ram2[30][6];
                int m1 = multi(pcm.ram1[28][4], v1 >> 8) >> 5;

                int v2 = addclip20(pcm.ram1[28][5], m1 >> 1, m1 & 1);
                pcm.ram1[28][5] = v2;
                int m2 = multi(v2, v1 & 255) >> 5;
                pcm.ram1[28][4] = addclip20(pcm.ram1[28][4], m2 >> 1, m2 & 1);


                pcm.ram1[29][4] = eram_unpack(&pcm, pcm.ram2[29][5] + pcm.tv_counter);
            }
            {
                // 10

                int v1 = pcm.ram2[30][6];
                int v2 = pcm.ram1[28][1];
                int m1 = multi(v2, v1 >> 8) >> 5;
                int s1 = eram_unpack(&pcm, pcm.ram2[28][8] + pcm.tv_counter);
                int v3 = addclip20(m1 >> 1, s1, m1 & 1);
                pcm.ram1[28][1] = v3;
                int m2 = multi(v3, v1 & 255) >> 5;
                pcm.ram1[29][5] = addclip20(m2 >> 1, v2, m2 & 1);

                eram_pack(&pcm, pcm.ram2[28][4] + pcm.tv_counter, pcm.ram1[28][3]);
            }
            {
                // 11

                int v1 = pcm.ram2[30][6];
                int v2 = pcm.ram1[29][4];
                int m1 = multi(v2, v1 >> 8) >> 5;
                int s1 = eram_unpack(&pcm, pcm.ram2[29][4] + pcm.tv_counter);
                int v3 = addclip20(m1 >> 1, s1, m1 & 1);
                pcm.ram1[29][4] = v3;
                int m2 = multi(v3, v1 & 255) >> 5;
                pcm.ram1[28][0] = addclip20(m2 >> 1, v2, m2 & 1);


                eram_pack(&pcm, pcm.ram2[28][5] + pcm.tv_counter, pcm.ram1[28][2]);

                eram_pack(&pcm, pcm.ram2[29][0] + pcm.tv_counter, pcm.ram1[28][5]);
            }
            {
                // 12

                pcm.ram1[28][5] = eram_unpack(&pcm, pcm.ram2[28][6] + pcm.tv_counter);
            }

            {
                // 13

                int s1 = eram_unpack(&pcm, pcm.ram2[28][10] + pcm.tv_counter);
                pcm.ram1[28][5] = addclip20(pcm.ram1[28][5], s1, 0);

                pcm.ram1[28][2] = eram_unpack(&pcm, pcm.ram2[29][2] + pcm.tv_counter);
            }

            {
                // 14

                int s1 = eram_unpack(&pcm, pcm.ram2[29][6] + pcm.tv_counter);
                int t1 = addclip20(s1, pcm.ram1[28][2], 0); // 6

                pcm.ram1[28][5] = addclip20(t1, pcm.ram1[28][5], 0);

                pcm.ram1[28][2] = eram_unpack(&pcm, pcm.ram2[28][7] + pcm.tv_counter);
            }

            {
                // 15

                int s1 = eram_unpack(&pcm, pcm.ram2[28][11] + pcm.tv_counter);
                pcm.ram1[28][2] = addclip20(pcm.ram1[28][2], s1, 0);

                pcm.ram1[28][3] = eram_unpack(&pcm, pcm.ram2[29][3] + pcm.tv_counter);
            }

            {
                // 16

                int s1 = eram_unpack(&pcm, pcm.ram2[29][7] + pcm.tv_counter);
                int t1 = addclip20(s1, pcm.ram1[28][2], 0);
                pcm.ram1[28][2] = addclip20(t1, pcm.ram1[28][3], 0);


                eram_pack(&pcm, pcm.ram2[29][1] + pcm.tv_counter, pcm.ram1[28][4]);

                eram_pack(&pcm, pcm.ram2[28][8] + pcm.tv_counter, pcm.ram1[28][1]);
            }

            {
                // 17
                int v1 = pcm.ram2[30][2];
                int v2 = pcm.ram1[28][5];

                int m1 = multi(v2, v1 >> 8) >> 5;

                rcadd[0] = m1;

                rcadd2[0] = multi(v2, v1 & 255) >> 5;

                int t1 = eram_unpack(&pcm, pcm.ram2[29][10] + pcm.tv_counter + 1); //? 3a6e
                eram_pack(&pcm, pcm.ram2[28][9] + pcm.tv_counter, pcm.ram1[29][5]);
                pcm.ram1[29][5] = t1;
            }

            {
                // 18
                int v1 = pcm.ram2[30][3];
                int v2 = pcm.ram1[28][2];

                int m1 = multi(v2, v1 >> 8) >> 5;

                rcadd[1] = m1;

                rcadd2[1] = multi(v2, v1 & 255) >> 5;

                pcm.ram1[28][1] = eram_unpack(&pcm, pcm.ram2[29][11] + pcm.tv_counter + 1); //? 3a1e
            }
            {
                // 19

                int v1 = pcm.ram2[31][9];

                int s1 = eram_unpack(&pcm, pcm.ram2[29][10] + pcm.tv_counter); //? 3a6d

                eram_pack(&pcm, pcm.ram2[29][4] + pcm.tv_counter, pcm.ram1[29][4]);

                int m1 = multi(s1, v1 >> 8) >> 5;
                int m2 = multi(pcm.ram1[29][5], v1 >> 8) >> 5;

                int t2 = addclip20(s1, (m1 >> 1) ^ 0xfffff, 1);

                pcm.ram1[29][5] = addclip20(t2, m2 >> 1, m2 & 1);
            }
            {
                // 20

                int v1 = pcm.ram2[31][10];

                int s1 = eram_unpack(&pcm, pcm.ram2[29][11] + pcm.tv_counter); //? 3a1d

                eram_pack(&pcm, pcm.ram2[29][5] + pcm.tv_counter, pcm.ram1[28][0]);

                int m1 = multi(s1, v1 >> 8) >> 5;
                int m2 = multi(pcm.ram1[28][1], v1 >> 8) >> 5;

                int t2 = addclip20(s1, (m1 >> 1) ^ 0xfffff, 1);

                pcm.ram1[28][1] = addclip20(t2, m2 >> 1, m2 & 1);

                eram_pack(&pcm, pcm.ram2[29][9] + pcm.tv_counter, pcm.ram1[29][1]);
            }
            {
                // 21

                int v1 = pcm.ram2[31][2];
                int v2 = pcm.ram1[29][5];

                int m1 = multi(v2, v1 >> 8) >> 5;
                int m2 = multi(v2, v1 & 255) >> 5;

                rcadd[2] = m1;
                rcadd2[2] = m2;
            }
            {
                // 22

                int v1 = pcm.ram2[31][3];
                int v2 = pcm.ram1[29][5];

                int m1 = multi(v2, v1 >> 8) >> 5;
                int m2 = multi(v2, v1 & 255) >> 5;

                rcadd[3] = m1;
                rcadd2[3] = m2;
            }
            {
                // 23

                int v1 = pcm.ram2[31][4];
                int v2 = pcm.ram1[28][1];

                int m1 = multi(v2, v1 >> 8) >> 5;
                int m2 = multi(v2, v1 & 255) >> 5;

                rcadd[4] = m1;
                rcadd2[4] = m2;
            }
            {
                // 31

                int v1 = pcm.ram2[31][5];
                int v2 = pcm.ram1[28][1];

                int m1 = multi(v2, v1 >> 8) >> 5;
                int m2 = multi(v2, v1 & 255) >> 5;

                rcadd[5] = m1;
                rcadd2[5] = m2;

                {
                    // address generator

                    int key = 1;
                    int okey = (pcm.ram2[31][7] & 0x20) != 0;
                    int active = key && okey;
                    int kon = key && !okey;

                    int b15 = (pcm.ram2[31][8] & 0x8000) != 0; // 0
                    int b6 = (pcm.ram2[31][7] & 0x40) != 0; // 1
                    int b7 = (pcm.ram2[31][7] & 0x80) != 0; // 1
                    int old_nibble = (pcm.ram2[31][7] >> 12) & 15; // 1

                    int address = pcm.ram1[31][4]; // 0
                    int address_end = pcm.ram1[31][0]; // 1 or 2
                    int address_loop = pcm.ram1[31][2]; // 2 or 1

                    int sub_phase = (pcm.ram2[31][8] & 0x3fff); // 1
                    int interp_ratio = (sub_phase >> 7) & 127;
                    sub_phase += pcm.ram2[pcm.ram2[31][7] & 31][0]; // 5
                    int sub_phase_of = (sub_phase >> 14) & 7;
                    if (pcm.nfs)
                    {
                        pcm.ram2[31][8] &= ~0x3fff;
                        pcm.ram2[31][8] |= sub_phase & 0x3fff;
                    }


                    // address 0
                    int address_cnt = address;

                    int cmp1 = b15 ? address_loop : address_end;
                    int cmp2 = address_cnt;
                    int address_cmp = (cmp1 & 0xfffff) == (cmp2 & 0xfffff); // 9
                    int next_b15 = b15;

                    int next_address = address_cnt; // 11

                    cmp1 = (!b6 && address_cmp) ? address_loop : address_cnt;
                    cmp2 = address_cnt;
                    int address_cnt2 = (kon || (!b6 && address_cmp)) ? cmp1 : cmp2;

                    int address_add = (!address_cmp && b6 && !b15) || (!address_cmp && !b6);
                    int address_sub = !address_cmp && b6 && b15;
                    if (b7)
                        address_cnt2 -= address_add - address_sub;
                    else
                        address_cnt2 += address_add - address_sub;
                    address_cnt = address_cnt2 & 0xfffff; // 11
                    b15 = b6 && (b15 ^ address_cmp); // 11

                    cmp1 = b15 ? address_loop : address_end;
                    cmp2 = address_cnt;
                    address_cmp = (cmp1 & 0xfffff) == (cmp2 & 0xfffff); // 13

                    if (sub_phase_of >= 1)
                    {
                        next_address = address_cnt; // 13
                        next_b15 = b15;
                    }

                    if (active && pcm.nfs)
                        pcm.ram1[31][4] = next_address;

                    if (pcm.nfs)
                    {
                        pcm.ram2[31][8] &= ~0x8000;
                        pcm.ram2[31][8] |= next_b15 << 15;
                    }

                    int t1 = address_loop; // 18
                    int t2 = pcm.ram1[31][4] - t1; // 19
                    int t3 = address_end - t2; // 20
                    int t4 = pcm.ram1[31][4]; // 23

                    pcm.ram2[29][10] = t3;
                    pcm.ram2[29][11] = t4;
                }
            }
        }

        pcm.ram1[31][1] = 0;
        pcm.ram1[31][3] = 0;
        pcm.rcsum[0] = 0;
        pcm.rcsum[1] = 0;

        // uint8_t irq_assert_per_slot[32] = {0};
        // uint8_t irq_channel_per_slot[32] = {0};
        // int sampl_per_slot[32] = {0};
        // int sampr_per_slot[32] = {0};
        // int rc0_per_slot[32] = {0};
        // int rc1_per_slot[32] = {0};

        for (int slot = 0; slot < reg_slots; slot++)
        {
            uint32_t *ram1 = pcm.ram1[slot];
            uint16_t *ram2 = pcm.ram2[slot];
            int okey = (ram2[7] & 0x20) != 0;
            int key = (voice_active >> slot) & 1;

            int active = okey && key;
            int kon = key && !okey;

            // address generator

            int b15 = (ram2[8] & 0x8000) != 0; // 0
            int b6 = (ram2[7] & 0x40) != 0; // 1
            int b7 = (ram2[7] & 0x80) != 0; // 1
            int hiaddr = (ram2[7] >> 8) & 15; // 1
            int old_nibble = (ram2[7] >> 12) & 15; // 1

            int address = ram1[4]; // 0
            int address_end = ram1[0]; // 1 or 2
            int address_loop = ram1[2]; // 2 or 1

            int cmp1 = b15 ? address_loop : address_end;
            int cmp2 = address;
            int nibble_cmp1 = (cmp1 & 0xffff0) == (cmp2 & 0xffff0); // 2
            int irq_flag = 0;

            // fixme:
            if (kon)
                irq_flag = ((cmp1 + address_loop) & 0x100000) != 0;
            else
                irq_flag = ((address + ((-address_loop) & 0xfffff)) & 0x100000) != 0;
            irq_flag ^= b7;

            int nibble_address = (!b6 && nibble_cmp1) ? address_loop : address; // 3
            int address_b4 = (nibble_address & 0x10) != 0;
            int wave_address = nibble_address >> 5;
            int xor2 = (address_b4 ^ b7);
            int check1 = xor2 && active;
            int xor1 = (b15 ^ !nibble_cmp1);
            int nibble_add = b6 ? check1 && xor1 : (!nibble_cmp1 && check1);
            int nibble_subtract = b6 && !xor1 && active && !xor2;
            if (b7)
                wave_address -= nibble_add - nibble_subtract;
            else
                wave_address += nibble_add - nibble_subtract;
            wave_address &= 0xfffff;

            int newnibble = PCM_ReadROM((hiaddr << 20) | wave_address);
            int newnibble_sel = address_b4 ^ ((b6 || !nibble_cmp1) && okey);
            if (newnibble_sel)
                newnibble = (newnibble >> 4) & 15;
            else
                newnibble &= 15;

            int sub_phase = (ram2[8] & 0x3fff); // 1
            int interp_ratio = (sub_phase >> 7) & 127;
            sub_phase += pcm.ram2[ram2[7] & 31][0]; // 5
            int sub_phase_of = (sub_phase >> 14) & 7;
            if (pcm.nfs)
            {
                ram2[8] &= ~0x3fff;
                ram2[8] |= sub_phase & 0x3fff;
            }


            // address 0
            int address_cnt = address;
            int samp0 = (int8_t)PCM_ReadROM((hiaddr << 20) | address_cnt); // 18

            cmp1 = address;
            cmp2 = address_cnt;
            int nibble_cmp2 = (cmp1 & 0xffff0) == (cmp2 & 0xffff0); // 8
            cmp1 = b15 ? address_loop : address_end;
            cmp2 = address_cnt;
            int address_cmp = (cmp1 & 0xfffff) == (cmp2 & 0xfffff); // 9

            int next_address = address_cnt; // 11
            int usenew = !nibble_cmp2;
            int next_b15 = b15;

            cmp1 = (!b6 && address_cmp) ? address_loop : address_cnt;
            cmp2 = address_cnt;
            int address_cnt2 = (kon || (!b6 && address_cmp)) ? cmp1 : cmp2;

            int address_add = (!address_cmp && b6 && !b15) || (!address_cmp && !b6);
            int address_sub = !address_cmp && b6 && b15;
            if (b7)
                address_cnt2 -= address_add - address_sub;
            else
                address_cnt2 += address_add - address_sub;
            address_cnt = address_cnt2 & 0xfffff; // 11
            b15 = b6 && (b15 ^ address_cmp); // 11

            int samp1 = (int8_t)PCM_ReadROM((hiaddr << 20) | address_cnt); // 20

            cmp1 = address;
            cmp2 = address_cnt;
            int nibble_cmp3 = (cmp1 & 0xffff0) == (cmp2 & 0xffff0); // 12
            cmp1 = b15 ? address_loop : address_end;
            cmp2 = address_cnt;
            address_cmp = (cmp1 & 0xfffff) == (cmp2 & 0xfffff); // 13

            if (sub_phase_of >= 1)
            {
                next_address = address_cnt; // 13
                usenew = !nibble_cmp3;
                next_b15 = b15;
            }

            cmp1 = (!b6 && address_cmp) ? address_loop : address_cnt;
            cmp2 = address_cnt;
            address_cnt2 = (kon || (!b6 && address_cmp)) ? cmp1 : cmp2;

            address_add = (!address_cmp && b6 && !b15) || (!address_cmp && !b6);
            address_sub = !address_cmp && b6 && b15;
            if (b7)
                address_cnt2 -= address_add - address_sub;
            else
                address_cnt2 += address_add - address_sub;
            address_cnt = address_cnt2 & 0xfffff; // 15
            b15 = b6 && (b15 ^ address_cmp); // 15

            int samp2 = (int8_t)PCM_ReadROM((hiaddr << 20) | address_cnt); // 1

            cmp1 = address;
            cmp2 = address_cnt;
            int nibble_cmp4 = (cmp1 & 0xffff0) == (cmp2 & 0xffff0); // 16
            cmp1 = b15 ? address_loop : address_end;
            cmp2 = address_cnt;
            address_cmp = (cmp1 & 0xfffff) == (cmp2 & 0xfffff); // 17

            if (sub_phase_of >= 2)
            {
                next_address = address_cnt; // 17
                usenew = !nibble_cmp4;
                next_b15 = b15;
            }

            cmp1 = (!b6 && address_cmp) ? address_loop : address_cnt;
            cmp2 = address_cnt;
            address_cnt2 = (kon || (!b6 && address_cmp)) ? cmp1 : cmp2;

            address_add = (!address_cmp && b6 && !b15) || (!address_cmp && !b6);
            address_sub = !address_cmp && b6 && b15;
            if (b7)
                address_cnt2 -= address_add - address_sub;
            else
                address_cnt2 += address_add - address_sub;
            address_cnt = address_cnt2 & 0xfffff; // 19
            b15 = b6 && (b15 ^ address_cmp); // 19

            int samp3 = (int8_t)PCM_ReadROM((hiaddr << 20) | address_cnt); // 5

            cmp1 = address;
            cmp2 = address_cnt;
            int nibble_cmp5 = (cmp1 & 0xffff0) == (cmp2 & 0xffff0); // 20
            cmp1 = b15 ? address_loop : address_end;
            cmp2 = address_cnt;
            address_cmp = (cmp1 & 0xfffff) == (cmp2 & 0xfffff); // 21

            if (sub_phase_of >= 3)
            {
                next_address = address_cnt; // 21
                usenew = !nibble_cmp5;
                next_b15 = b15;
            }

            cmp1 = (!b6 && address_cmp) ? address_loop : address_cnt;
            cmp2 = address_cnt;
            address_cnt2 = (kon || (!b6 && address_cmp)) ? cmp1 : cmp2;

            address_add = (!address_cmp && b6 && !b15) || (!address_cmp && !b6);
            address_sub = !address_cmp && b6 && b15;
            if (b7)
                address_cnt2 -= address_add - address_sub;
            else
                address_cnt2 += address_add - address_sub;
            address_cnt = address_cnt2 & 0xfffff; // 23
            // b15 = b6 && (b15 ^ address_cmp); // 23

            cmp1 = address;
            cmp2 = address_cnt;
            int nibble_cmp6 = (cmp1 & 0xffff0) == (cmp2 & 0xffff0); // 24

            if (sub_phase_of >= 4)
            {
                next_address = address_cnt; // 1
                usenew = !nibble_cmp6;
                // b15 is not updated?
            }

            if (active && pcm.nfs)
                ram1[4] = next_address;

            if (pcm.nfs)
            {
                ram2[8] &= ~0x8000;
                ram2[8] |= next_b15 << 15;
            }

            // dpcm

            // 18
            int reference = ram1[5];

            // 19
            int preshift = samp0 << 10;
            int select_nibble = nibble_cmp2 ? old_nibble : newnibble;
            int shift = (10 - select_nibble) & 15;

            int shifted = (preshift << 1) >> shift;

            if (sub_phase_of >= 1)
                reference = addclip20(reference, shifted >> 1, shifted & 1);

            preshift = samp1 << 10;
            select_nibble = nibble_cmp3 ? old_nibble : newnibble;
            shift = (10 - select_nibble) & 15;

            shifted = (preshift << 1) >> shift;

            if (sub_phase_of >= 2)
                reference = addclip20(reference, shifted >> 1, shifted & 1);

            preshift = samp2 << 10;
            select_nibble = nibble_cmp4 ? old_nibble : newnibble;
            shift = (10 - select_nibble) & 15;

            shifted = (preshift << 1) >> shift;

            if (sub_phase_of >= 3)
                reference = addclip20(reference, shifted >> 1, shifted & 1);

            preshift = samp3 << 10;
            select_nibble = nibble_cmp5 ? old_nibble : newnibble;
            shift = (10 - select_nibble) & 15;

            shifted = (preshift << 1) >> shift;

            if (sub_phase_of >= 4)
                reference = addclip20(reference, shifted >> 1, shifted & 1);

            // interpolation

            int test = ram1[5];

            int step0 = multi(interp_lut[0][interp_ratio] << 6, samp0) >> 8;
            select_nibble = nibble_cmp2 ? old_nibble : newnibble;
            shift = (10 - select_nibble) & 15;
            step0 =  (step0 << 1) >> shift;

            test = addclip20(test, step0 >> 1, step0 & 1);


            int step1 = multi(interp_lut[1][interp_ratio] << 6, samp1) >> 8;
            select_nibble = nibble_cmp3 ? old_nibble : newnibble;
            shift = (10 - select_nibble) & 15;
            step1 = (step1 << 1) >> shift;

            test = addclip20(test, step1 >> 1, step1 & 1);

            int step2 = multi(interp_lut[2][interp_ratio] << 6, samp2) >> 8;
            select_nibble = nibble_cmp4 ? old_nibble : newnibble;
            shift = (10 - select_nibble) & 15;
            step2 = (step2 << 1) >> shift;

            int reg1 = ram1[1];
            int reg3 = ram1[3];
            int reg2_6 = (ram2[6] >> 8) & 127;

            test = addclip20(test, step2 >> 1, step2 & 1);

            int filter = ram2[11];
            int v3;

            // if (mcu->mcu_mk1)
            if (false) 
            {
                int mult1 = multi(reg1, filter >> 8); // 8
                int mult2 = multi(reg1, (filter >> 1) & 127); // 9
                int mult3 = multi(reg1, reg2_6); // 10

                int v2 = addclip20(reg3, mult1 >> 6, (mult1 >> 5) & 1); // 9
                int v1 = addclip20(v2, mult2 >> 13, (mult2 >> 12) & 1); // 10
                int subvar = addclip20(v1, (mult3 >> 6), (mult3 >> 5) & 1); // 11

                ram1[3] = v1;

                v3 = addclip20(test, subvar ^ 0xfffff, 1); // 12

                int mult4 = multi(v3, filter >> 8);
                int mult5 = multi(v3, (filter >> 1) & 127);
                int v4 = addclip20(reg1, mult4 >> 6, (mult4 >> 5) & 1); // 14
                int v5 = addclip20(v4, mult5 >> 13, (mult5 >> 12) & 1); // 15

                ram1[1] = v5;
            }
            else
            {
                // hack: use 32-bit math to avoid overflow
                int mult1 = reg1 * (int8_t)(filter >> 8); // 8
                int mult2 = reg1 * (int8_t)((filter >> 1) & 127); // 9
                int mult3 = reg1 * (int8_t)reg2_6; // 10

                int v2 = reg3 + (mult1 >> 6) + ((mult1 >> 5) & 1); // 9
                int v1 = v2 + (mult2 >> 13) + ((mult2 >> 12) & 1); // 10
                int subvar = v1 + (mult3 >> 6) + ((mult3 >> 5) & 1); // 11
#define Clamp(v, x0, x1) (v < x0 ? x0 : v > x1 ? x1 : v)

                ram1[3] = Clamp(v1, -0x80000, 0x7ffff);

                int tests = test;
                tests <<= 12;
                tests >>= 12;

                v3 = tests - subvar; // 12

                int mult4 = v3 * (int8_t)(filter >> 8);
                int mult5 = v3 * (int8_t)((filter >> 1) & 127);
                int v4 = reg1 + (mult4 >> 6) + ((mult4 >> 5) & 1); // 14
                int v5 = v4 + (mult5 >> 13) + ((mult5 >> 12) & 1); // 15

                ram1[1] = Clamp(v5, -0x80000, 0x7ffff);
            }


            ram1[5] = reference;

            if (active && (ram2[6] & 1) != 0 && (ram2[8] & 0x4000) == 0 && !pcm.irq_assert && irq_flag)
            {
                //printf("irq voice %i\n", slot);
                if (pcm.nfs)
                    ram2[8] |= 0x4000;
                pcm.irq_assert = 1;
                pcm.irq_channel = slot;
                mcu->MCU_GA_SetGAInt(5, 1);
            }

            int volmul1 = 0;
            int volmul2 = 0;

            calc_tv(&pcm, 0, ram2[3], &ram2[9], active, &volmul1);
            calc_tv(&pcm, 1, ram2[4], &ram2[10], active, &volmul2);
            calc_tv(&pcm, 2, ram2[5], &ram2[11], active, NULL);

            // if (volmul1 && volmul2)
            //     volmul1 += 0;

            int sample = (ram2[6] & 2) == 0 ? ram1[3] : v3;
            //sample = test;

            int multiv1 = multi(sample, volmul1 >> 8);
            int multiv2 = multi(sample, (volmul1 >> 1) & 127);

            int sample2 = addclip20(multiv1 >> 6, multiv2 >> 13, ((multiv2 >> 12) | (multiv1 >> 5)) & 1);

            int multiv3 = multi(sample2, volmul2 >> 8);
            int multiv4 = multi(sample2, (volmul2 >> 1) & 127);

            int sample3 = addclip20(multiv3 >> 6, multiv4 >> 13, ((multiv4 >> 12) | (multiv3 >> 5)) & 1);

            int pan = active ? ram2[1] : 0;
            int rc = active ? ram2[2] : 0;

            int sampl = multi(sample3, (pan >> 8) & 255);
            int sampr = multi(sample3, (pan >> 0) & 255);

            int rc0 = multi(sample3, (rc >> 8) & 255) >> 5; // reverb
            int rc1 = multi(sample3, (rc >> 0) & 255) >> 5; // chorus

            // sampl_per_slot[slot] = sampl;
            // sampr_per_slot[slot] = sampr;
            // rc0_per_slot[slot] = rc0;
            // rc1_per_slot[slot] = rc1;

            if (key && pcm.nfs)
            {
                ram2[7] &= ~0xf020;
                ram2[7] |= ((usenew || kon) ? newnibble : old_nibble) << 12;

                // update key
                ram2[7] |= key << 5;
            }

            if (!active)
            {
                if (pcm.nfs)
                {
                    ram1[1] = 0;
                    ram1[3] = 0;
                    ram1[5] = 0;
                }

                ram2[8] = 0;
                ram2[9] = 0;
                ram2[10] = 0;
            }
        // }
            
        // for (int slot = 0; slot < reg_slots; slot++)
        // {
        //     int sampl = sampl_per_slot[slot];
        //     int sampr = sampr_per_slot[slot];
        //     int rc0 = rc0_per_slot[slot];
        //     int rc1 = rc1_per_slot[slot];

            // mix reverb/chorus?
            int next_slot = (slot == reg_slots - 1) ? 31 : slot + 1;
            switch (next_slot)
            {
                // 17, 18 - reverb

                case 17: {
                    pcm.ram1[31][1] = addclip20(pcm.ram1[31][1], rcadd[0] >> 1, rcadd[0] & 1);
                    pcm.rcsum[1] = addclip20(pcm.rcsum[1], rcadd2[0] >> 1, rcadd2[0] & 1);
                    break;
                }
                case 18: {
                    pcm.ram1[31][3] = addclip20(pcm.ram1[31][3], rcadd[1] >> 1, rcadd[1] & 1);
                    pcm.rcsum[1] = addclip20(pcm.rcsum[1], rcadd2[1] >> 1, rcadd2[1] & 1);
                    break;
                }
                case 21: {
                    pcm.ram1[31][1] = addclip20(pcm.ram1[31][1], rcadd[2] >> 1, rcadd[2] & 1);
                    pcm.rcsum[0] = addclip20(pcm.rcsum[0], rcadd2[2] >> 1, rcadd2[2] & 1);
                    break;
                }
                case 22: {
                    pcm.ram1[31][3] = addclip20(pcm.ram1[31][3], rcadd[3] >> 1, rcadd[3] & 1);
                    pcm.rcsum[1] = addclip20(pcm.rcsum[1], rcadd2[3] >> 1, rcadd2[3] & 1);
                    break;
                }
                case 23: {
                    pcm.ram1[31][1] = addclip20(pcm.ram1[31][1], rcadd[4] >> 1, rcadd[4] & 1);
                    pcm.rcsum[0] = addclip20(pcm.rcsum[0], rcadd2[4] >> 1, rcadd2[4] & 1);
                    break;
                }
                case 31: {
                    pcm.ram1[31][3] = addclip20(pcm.ram1[31][3], rcadd[5] >> 1, rcadd[5] & 1);
                    pcm.rcsum[1] = addclip20(pcm.rcsum[1], rcadd2[5] >> 1, rcadd2[5] & 1);
                    break;
                }
            }

            pcm.rcsum[0] = addclip20(pcm.rcsum[0], rc0 >> 1, rc0 & 1);
            pcm.rcsum[1] = addclip20(pcm.rcsum[1], rc1 >> 1, rc1 & 1);

            int suml = addclip20(pcm.ram1[31][1], sampl >> 6, (sampl >> 5) & 1);
            int sumr = addclip20(pcm.ram1[31][3], sampr >> 6, (sampr >> 5) & 1);
            if (slot != reg_slots - 1)
            {
                pcm.ram1[31][1] = suml;
                pcm.ram1[31][3] = sumr;
            }
            else
            {
                pcm.accum_l = suml;
                pcm.accum_r = sumr;
            }
        }

        if (pcm.nfs)
        {
            pcm.ram2[31][7] |= 0x20;
        }

         pcm_lock.Release();

        pcm.nfs = 1;

        constexpr int cycles = (reg_slots + 1) * 25;
        pcm.cycles += (cycles * 25) / 29;
    }
}
*/

void Pcm::PCM_Update(uint64_t cycles_target)
{
    // ---- быстрые константы/алиасы
    constexpr int REG_SLOTS         = 28;                 // было вычисляемым, но у тебя закреплено 28
    constexpr int WRITE_MASK        = 3;
    constexpr int CYCLE_A           = (REG_SLOTS + 1) * 25;
    constexpr int CYCLES_PER_SAMPLE = (CYCLE_A * 25) / 29; // как в оригинале, только вынесено
    constexpr int BLOCK_SIZE        = 32;                 // блоковая обработка (можно 64/128 по месту)

    // Схлопываем маску активных голосов заранее
    const int voice_active_all = (pcm.voice_mask & pcm.voice_mask_pending);

    // Будем обрабатывать пачками семплов, чтобы реже входить в внешний while
    while (pcm.cycles < cycles_target)
    {
        // Сколько семплов реально можно посчитать в этом вызове
        const uint64_t cycles_left   = cycles_target - pcm.cycles;
        int samples_this_block       = (int)(cycles_left / CYCLES_PER_SAMPLE);
        if (samples_this_block <= 0) samples_this_block = 1;
        if (samples_this_block > BLOCK_SIZE) samples_this_block = BLOCK_SIZE;

        // -------- основной «внутренний» цикл по семплам (блочно)
        for (int sample_i = 0; sample_i < samples_this_block; ++sample_i)
        {

            pcm_lock.Acquire();
            // ---- финальный микс/шум LFSR: компактнее, без лишних временных
            {
                int shifter = pcm.ram2[30][10];
                // шаг LFSR
                int xr = ((shifter >> 0) ^ (shifter >> 1) ^ (shifter >> 7) ^ (shifter >> 12)) & 1;
                shifter = (shifter >> 1) | (xr << 15);
                pcm.ram2[30][10] = shifter;

                // микс левого/правого аккума (без опций oversampling: у тебя это всегда false)
                pcm.accum_l = addclip20(pcm.accum_l, pcm.ram1[30][0], 0);
                pcm.accum_r = addclip20(pcm.accum_r, pcm.ram1[30][1], 0);

                // noise_mask/orval всегда 0 в твоём коде — оставляю, чтобы не ломать логику
                // (если позже включишь, просто поставишь значения в начале блока)
                const int noise_mask = 0;
                const int orval      = 0;

                pcm.ram1[30][2] = addclip20(pcm.accum_l, (orval | (shifter & noise_mask)), 0);
                pcm.ram1[30][4] = addclip20(pcm.accum_r, (orval | (shifter & noise_mask)), 0);

                // write-back с маской
                pcm.ram1[30][0] = (pcm.accum_l & WRITE_MASK);
                pcm.ram1[30][1] = (pcm.accum_r & WRITE_MASK);

                int outL = (int)((pcm.ram1[30][2] & ~WRITE_MASK) << 12);
                int outR = (int)((pcm.ram1[30][4] & ~WRITE_MASK) << 12);
                int tt[2] = { outL, outR };
                mcu->MCU_PostSample(tt);

                // второй полушаг LFSR
                xr      = ((shifter >> 0) ^ (shifter >> 1) ^ (shifter >> 7) ^ (shifter >> 12)) & 1;
                shifter = (shifter >> 1) | (xr << 15);

                pcm.accum_l = addclip20(pcm.accum_l, pcm.ram1[30][0], 0);
                pcm.accum_r = addclip20(pcm.accum_r, pcm.ram1[30][1], 0);

                pcm.ram1[30][3] = addclip20(pcm.accum_l, (orval | (shifter & noise_mask)), 0);
                pcm.ram1[30][5] = addclip20(pcm.accum_r, (orval | (shifter & noise_mask)), 0);
            }

            // ---- глобальный счётчик огибающих/TV
            {
                if (!pcm.nfs)
                    pcm.tv_counter = pcm.ram2[31][8]; // fixme сохранил как есть

                // modulo 0x4000
                pcm.tv_counter = (pcm.tv_counter - 1) & 0x3fff;
            }

            // ---- «хор/реверб» подготовка (оставил семантику неизменной, схлопнул if/else)
            {
                const int r318 = pcm.ram2[31][8];
                const int a    = (r318 & 0x7fff);
                const int b    = ((0x4000 - r318) & 0x7fff);

                if (r318 & 0x8000) pcm.ram2[31][9]  = a; else pcm.ram2[31][10] = a;
                if ((0x4000 - r318) & 0x8000) pcm.ram2[31][10] = b; else pcm.ram2[31][9]  = b;
            }

            // два мелких блока свёл к минимуму временных
            {
                const int v1 = pcm.ram2[31][1];
                const int m1 = (multi(pcm.ram1[29][1], v1 >> 8) >> 5);
                const int m2 = (multi(pcm.rcsum[1],  v1 & 255) >> 5);
                pcm.ram1[29][1] = addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1);
            }
            {
                const int okey   = (pcm.ram2[31][7] & 0x20) != 0;
                const int active = (okey ); // key=1 в исходнике
                int u = 0;
                calc_tv(&pcm, 1, pcm.ram2[30][0], &pcm.ram2[30][9], active, &u);
            }
            {
                const int v1 = pcm.ram2[30][1];
                const int m1 = (multi(pcm.ram1[29][0], (v1 >> 8)) >> 5);
                const int m2 = (multi(pcm.rcsum[0],    v1 & 255) >> 5);
                pcm.ram1[29][0] = addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1);
            }

            // ---- временные накопители для RC (сводим на стек единоразово)
            int rcadd [6] = {0,0,0,0,0,0};
            int rcadd2[6] = {0,0,0,0,0,0};

            // ---- большой «FX» блок: оставлен по шагам 1..23..31,
            //     но убраны лишние временные и повторные чтения.
            {
                // 1
                {
                    const int v1 = pcm.ram2[30][4];
                    const int m1 = (multi(pcm.ram1[29][0], (v1 >> 8)) >> 6);
                    const int s1 = eram_unpack(&pcm, pcm.ram2[28][1] + pcm.tv_counter, 1);
                    const int s2 = eram_unpack(&pcm, pcm.ram2[28][1] + pcm.tv_counter);
                    const int v2 = ((v1 & 0x30) != 0) ? s1 : 0;
                    const int v3 = addclip20(m1, (v2 ^ 0xfffff), 1);
                    pcm.ram1[29][4] = v3;
                    const int m2 = (multi(v3, v1 & 255) >> 5);
                    pcm.ram1[29][5] = addclip20(m2 >> 1, s2, m2 & 1);
                }
                // 2
                {
                    const int v1 = pcm.ram2[30][4];
                    const int s1 = eram_unpack(&pcm, pcm.ram2[28][2] + pcm.tv_counter, 1);
                    const int s2 = eram_unpack(&pcm, pcm.ram2[28][2] + pcm.tv_counter);
                    const int v2 = ((v1 & 0x30) != 0) ? s1 : 0;
                    const int v3 = addclip20(pcm.ram1[29][5], (v2 ^ 0xfffff), 1);
                    pcm.ram1[29][5] = v3;
                    const int m2 = (multi(v3, v1 & 255) >> 5);
                    pcm.ram1[28][0] = addclip20(m2 >> 1, s2, m2 & 1);
                }
                // 3
                {
                    const int v1 = pcm.ram2[30][4];
                    const int s1 = eram_unpack(&pcm, pcm.ram2[28][3] + pcm.tv_counter, 1);
                    const int s2 = eram_unpack(&pcm, pcm.ram2[28][3] + pcm.tv_counter);
                    const int v2 = ((v1 & 0x30) != 0) ? s1 : 0;
                    const int v3 = addclip20(pcm.ram1[28][0], (v2 ^ 0xfffff), 1);
                    pcm.ram1[28][0] = v3;
                    const int m2 = (multi(v3, v1 & 255) >> 5);
                    pcm.ram1[28][1] = addclip20(m2 >> 1, s2, m2 & 1);
                    pcm.ram1[28][2] = eram_unpack(&pcm, pcm.ram2[28][5] + pcm.tv_counter);
                }
                // 4
                {
                    const int v1 = pcm.ram2[30][5];
                    const int s1 = eram_unpack(&pcm, pcm.ram2[28][4] + pcm.tv_counter, 1);
                    const int s2 = eram_unpack(&pcm, pcm.ram2[28][4] + pcm.tv_counter);
                    const int v2 = ((v1 & 0x30) != 0) ? s1 : 0;
                    const int v3 = addclip20(pcm.ram1[28][1], (v2 ^ 0xfffff), 1);
                    pcm.ram1[28][1] = v3;
                    const int m2 = (multi(v3, v1 & 255) >> 5);
                    pcm.ram1[28][3] = addclip20(m2 >> 1, s2, m2 & 1);
                    pcm.ram1[28][4] = eram_unpack(&pcm, pcm.ram2[29][1] + pcm.tv_counter);
                }
                // 5
                {
                    const int v1 = pcm.ram2[30][7];
                    const int m1 = (multi(pcm.ram1[29][2], (v1 >> 8)) >> 5);
                    const int s1 = eram_unpack(&pcm, pcm.ram2[29][0] + pcm.tv_counter);
                    const int m2 = (multi(s1, v1 & 255) >> 5);
                    pcm.ram1[29][2] = addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1);
                    eram_pack(&pcm, pcm.ram2[28][0] + pcm.tv_counter, pcm.ram1[29][4]);
                }
                // 6
                {
                    const int v1 = pcm.ram2[30][8];
                    const int m1 = (multi(pcm.ram1[29][3], (v1 >> 8)) >> 5);
                    const int s1 = eram_unpack(&pcm, pcm.ram2[29][8] + pcm.tv_counter);
                    const int m2 = (multi(s1, v1 & 255) >> 5);
                    pcm.ram1[29][3] = addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1);
                    eram_pack(&pcm, pcm.ram2[28][1] + pcm.tv_counter, pcm.ram1[29][5]);
                    eram_pack(&pcm, pcm.ram2[28][2] + pcm.tv_counter, pcm.ram1[28][0]);
                }
                // 7
                {
                    const int v1 = pcm.ram2[30][9];
                    const int v2 = pcm.ram1[28][3];
                    const int m1 = (multi(pcm.ram1[29][2], (v1 >> 8)) >> 5);
                    const int m2 = (multi(pcm.ram1[29][3], (v1 >> 8)) >> 5);
                    pcm.ram1[28][3] = addclip20(v2, m1 >> 1, m1 & 1);
                    pcm.ram1[28][5] = addclip20(v2, m2 >> 1, m2 & 1);
                    eram_pack(&pcm, pcm.ram2[28][3] + pcm.tv_counter, pcm.ram1[28][1]);
                }
                // 8
                {
                    const int v1 = pcm.ram2[30][6];
                    const int m1 = (multi(pcm.ram1[28][2], v1 >> 8) >> 5);
                    const int v2 = addclip20(pcm.ram1[28][3], m1 >> 1, m1 & 1);
                    pcm.ram1[28][3] = v2;
                    const int m2 = (multi(v2, v1 & 255) >> 5);
                    pcm.ram1[28][2] = addclip20(pcm.ram1[28][2], m2 >> 1, m2 & 1);
                    pcm.ram1[28][1] = eram_unpack(&pcm, pcm.ram2[28][9] + pcm.tv_counter);
                }
                // 9
                {
                    const int v1 = pcm.ram2[30][6];
                    const int m1 = (multi(pcm.ram1[28][4], v1 >> 8) >> 5);
                    const int v2 = addclip20(pcm.ram1[28][5], m1 >> 1, m1 & 1);
                    pcm.ram1[28][5] = v2;
                    const int m2 = (multi(v2, v1 & 255) >> 5);
                    pcm.ram1[28][4] = addclip20(pcm.ram1[28][4], m2 >> 1, m2 & 1);
                    pcm.ram1[29][4] = eram_unpack(&pcm, pcm.ram2[29][5] + pcm.tv_counter);
                }
                // 10
                {
                    const int v1 = pcm.ram2[30][6];
                    const int v2 = pcm.ram1[28][1];
                    const int m1 = (multi(v2, v1 >> 8) >> 5);
                    const int s1 = eram_unpack(&pcm, pcm.ram2[28][8] + pcm.tv_counter);
                    const int v3 = addclip20(m1 >> 1, s1, m1 & 1);
                    pcm.ram1[28][1] = v3;
                    const int m2 = (multi(v3, v1 & 255) >> 5);
                    pcm.ram1[29][5] = addclip20(m2 >> 1, v2, m2 & 1);
                    eram_pack(&pcm, pcm.ram2[28][4] + pcm.tv_counter, pcm.ram1[28][3]);
                }
                // 11
                {
                    const int v1 = pcm.ram2[30][6];
                    const int v2 = pcm.ram1[29][4];
                    const int m1 = (multi(v2, v1 >> 8) >> 5);
                    const int s1 = eram_unpack(&pcm, pcm.ram2[29][4] + pcm.tv_counter);
                    const int v3 = addclip20(m1 >> 1, s1, m1 & 1);
                    pcm.ram1[29][4] = v3;
                    const int m2 = (multi(v3, v1 & 255) >> 5);
                    pcm.ram1[28][0] = addclip20(m2 >> 1, v2, m2 & 1);
                    eram_pack(&pcm, pcm.ram2[28][5] + pcm.tv_counter, pcm.ram1[28][2]);
                    eram_pack(&pcm, pcm.ram2[29][0] + pcm.tv_counter, pcm.ram1[28][5]);
                }
                // 12
                {
                    pcm.ram1[28][5] = eram_unpack(&pcm, pcm.ram2[28][6] + pcm.tv_counter);
                }
                // 13
                {
                    const int s1 = eram_unpack(&pcm, pcm.ram2[28][10] + pcm.tv_counter);
                    pcm.ram1[28][5] = addclip20(pcm.ram1[28][5], s1, 0);
                    pcm.ram1[28][2] = eram_unpack(&pcm, pcm.ram2[29][2] + pcm.tv_counter);
                }
                // 14
                {
                    const int s1 = eram_unpack(&pcm, pcm.ram2[29][6] + pcm.tv_counter);
                    const int t1 = addclip20(s1, pcm.ram1[28][2], 0);
                    pcm.ram1[28][5] = addclip20(t1, pcm.ram1[28][5], 0);
                    pcm.ram1[28][2] = eram_unpack(&pcm, pcm.ram2[28][7] + pcm.tv_counter);
                }
                // 15
                {
                    const int s1 = eram_unpack(&pcm, pcm.ram2[28][11] + pcm.tv_counter);
                    pcm.ram1[28][2] = addclip20(pcm.ram1[28][2], s1, 0);
                    pcm.ram1[28][3] = eram_unpack(&pcm, pcm.ram2[29][3] + pcm.tv_counter);
                }
                // 16
                {
                    const int s1 = eram_unpack(&pcm, pcm.ram2[29][7] + pcm.tv_counter);
                    const int t1 = addclip20(s1, pcm.ram1[28][2], 0);
                    pcm.ram1[28][2] = addclip20(t1, pcm.ram1[28][3], 0);
                    eram_pack(&pcm, pcm.ram2[29][1] + pcm.tv_counter, pcm.ram1[28][4]);
                    eram_pack(&pcm, pcm.ram2[28][8] + pcm.tv_counter, pcm.ram1[28][1]);
                }
                // 17
                {
                    const int v1 = pcm.ram2[30][2];
                    const int v2 = pcm.ram1[28][5];
                    const int m1 = (multi(v2, v1 >> 8) >> 5);
                    rcadd [0] = m1;
                    rcadd2[0] = (multi(v2, v1 & 255) >> 5);
                    const int t1 = eram_unpack(&pcm, pcm.ram2[29][10] + pcm.tv_counter + 1);
                    eram_pack(&pcm, pcm.ram2[28][9] + pcm.tv_counter, pcm.ram1[29][5]);
                    pcm.ram1[29][5] = t1;
                }
                // 18
                {
                    const int v1 = pcm.ram2[30][3];
                    const int v2 = pcm.ram1[28][2];
                    const int m1 = (multi(v2, v1 >> 8) >> 5);
                    rcadd [1] = m1;
                    rcadd2[1] = (multi(v2, v1 & 255) >> 5);
                    pcm.ram1[28][1] = eram_unpack(&pcm, pcm.ram2[29][11] + pcm.tv_counter + 1);
                }
                // 19
                {
                    const int v1 = pcm.ram2[31][9];
                    const int s1 = eram_unpack(&pcm, pcm.ram2[29][10] + pcm.tv_counter);
                    eram_pack(&pcm, pcm.ram2[29][4] + pcm.tv_counter, pcm.ram1[29][4]);
                    const int m1 = (multi(s1, v1 >> 8) >> 5);
                    const int m2 = (multi(pcm.ram1[29][5], v1 >> 8) >> 5);
                    const int t2 = addclip20(s1, ((m1 >> 1) ^ 0xfffff), 1);
                    pcm.ram1[29][5] = addclip20(t2, (m2 >> 1), (m2 & 1));
                }
                // 20
                {
                    const int v1 = pcm.ram2[31][10];
                    const int s1 = eram_unpack(&pcm, pcm.ram2[29][11] + pcm.tv_counter);
                    eram_pack(&pcm, pcm.ram2[29][5] + pcm.tv_counter, pcm.ram1[28][0]);
                    const int m1 = (multi(s1, v1 >> 8) >> 5);
                    const int m2 = (multi(pcm.ram1[28][1], v1 >> 8) >> 5);
                    const int t2 = addclip20(s1, ((m1 >> 1) ^ 0xfffff), 1);
                    pcm.ram1[28][1] = addclip20(t2, (m2 >> 1), (m2 & 1));
                    eram_pack(&pcm, pcm.ram2[29][9] + pcm.tv_counter, pcm.ram1[29][1]);
                }
                // 21
                {
                    const int v1 = pcm.ram2[31][2];
                    const int v2 = pcm.ram1[29][5];
                    rcadd [2] = (multi(v2, v1 >> 8) >> 5);
                    rcadd2[2] = (multi(v2, v1 & 255) >> 5);
                }
                // 22
                {
                    const int v1 = pcm.ram2[31][3];
                    const int v2 = pcm.ram1[29][5];
                    rcadd [3] = (multi(v2, v1 >> 8) >> 5);
                    rcadd2[3] = (multi(v2, v1 & 255) >> 5);
                }
                // 23
                {
                    const int v1 = pcm.ram2[31][4];
                    const int v2 = pcm.ram1[28][1];
                    rcadd [4] = (multi(v2, v1 >> 8) >> 5);
                    rcadd2[4] = (multi(v2, v1 & 255) >> 5);
                }
                // 31
                {
                    const int v1 = pcm.ram2[31][5];
                    const int v2 = pcm.ram1[28][1];
                    rcadd [5] = (multi(v2, v1 >> 8) >> 5);
                    rcadd2[5] = (multi(v2, v1 & 255) >> 5);

                    // address generator (общий, как в исходнике, key=1)
                    const int key  = 1;
                    const int okey = (pcm.ram2[31][7] & 0x20) != 0;
                    const int active = (key && okey);
                    const int kon    = (key && !okey);

                    bool b15   = (pcm.ram2[31][8] & 0x8000) != 0;
                    const bool b6    = (pcm.ram2[31][7] & 0x40) != 0;
                    const bool b7    = (pcm.ram2[31][7] & 0x80) != 0;

                    const int address_end  = pcm.ram1[31][0];
                    const int address_loop = pcm.ram1[31][2];

                    int sub_phase   = (pcm.ram2[31][8] & 0x3fff);
                    const int step  = pcm.ram2[ pcm.ram2[31][7] & 31 ][0];
                    sub_phase      += step;
                    const int sub_phase_of = (sub_phase >> 14) & 7;
                    if (pcm.nfs) {
                        pcm.ram2[31][8] = (pcm.ram2[31][8] & ~0x3fff) | (sub_phase & 0x3fff);
                    }

                    // address 0
                    int address_cnt = pcm.ram1[31][4];

                    const int cmp1a = b15 ? address_loop : address_end;
                    bool address_cmp = ((cmp1a & 0xfffff) == (address_cnt & 0xfffff));
                    int next_address = address_cnt;
                    bool next_b15    = b15;

                    int address_cnt2 = (kon || (!b6 && address_cmp)) ? ( (!b6 && address_cmp) ? address_loop : address_cnt ) : address_cnt;

                    const int address_add = ((!address_cmp && b6 && !b15) || (!address_cmp && !b6)) ? 1 : 0;
                    const int address_sub = (!address_cmp && b6 && b15) ? 1 : 0;
                    address_cnt = b7 ? ((address_cnt2 - (address_add - address_sub)) & 0xfffff)
                                     : ((address_cnt2 + (address_add - address_sub)) & 0xfffff);
                    b15 = b6 && (b15 ^ address_cmp);

                    const int cmp1b = b15 ? address_loop : address_end;
                    address_cmp = ((cmp1b & 0xfffff) == (address_cnt & 0xfffff));

                    if (sub_phase_of >= 1) {
                        next_address = address_cnt;
                        next_b15     = b15;
                    }
                    if (active && pcm.nfs) pcm.ram1[31][4] = next_address;
                    if (pcm.nfs) {
                        pcm.ram2[31][8] = (pcm.ram2[31][8] & ~0x8000) | (int(next_b15) << 15);
                    }

                    const int t1 = address_loop;
                    const int t2 = pcm.ram1[31][4] - t1;
                    const int t3 = address_end - t2;
                    const int t4 = pcm.ram1[31][4];

                    pcm.ram2[29][10] = t3;
                    pcm.ram2[29][11] = t4;
                }
            }

            // ---- подготовка сумматоров перед проходом по слотам
            pcm.ram1[31][1] = 0;
            pcm.ram1[31][3] = 0;
            pcm.rcsum[0] = 0;
            pcm.rcsum[1] = 0;

            // ---- основной цикл по слотам
            for (int slot = 0; slot < REG_SLOTS; ++slot)
            {
                uint32_t* __restrict ram1 = pcm.ram1[slot];
                uint16_t* __restrict ram2 = pcm.ram2[slot];

                const int okey  = (ram2[7] & 0x20) != 0;
                const int key   = (voice_active_all >> slot) & 1;
                const int active = (okey & key);
                const int kon    = (key & !okey);

                // address generator (перенёс локальные константы выше, минимизировал повтор)
                bool b15     = (ram2[8] & 0x8000) != 0;
                const bool b6      = (ram2[7] & 0x40) != 0;
                const bool b7      = (ram2[7] & 0x80) != 0;
                const int  hiaddr  = (ram2[7] >> 8) & 15;
                const int  old_nib = (ram2[7] >> 12) & 15;

                const int address_end  = ram1[0];
                const int address_loop = ram1[2];
                int       address      = ram1[4];

                const int cmp1          = b15 ? address_loop : address_end;
                const bool nibble_cmp1  = ((cmp1 & 0xffff0) == (address & 0xffff0));

                // irq_flag (сохранил исходную формулу)
                int irq_flag;
                if (kon)
                    irq_flag = (((cmp1 + address_loop) & 0x100000) != 0);
                else
                    irq_flag = (((address + ((-address_loop) & 0xfffff)) & 0x100000) != 0);
                irq_flag ^= int(b7);

                int nibble_address = (!b6 && nibble_cmp1) ? address_loop : address;
                const int address_b4 = (nibble_address & 0x10) != 0;
                int wave_address = (nibble_address >> 5) & 0xfffff;
                const int xor2 = (address_b4 ^ int(b7));
                const int check1 = (xor2 & active);
                const int xor1 = (int(b15) ^ int(!nibble_cmp1));
                const int nibble_add      = b6 ? (check1 && xor1) : (!nibble_cmp1 && check1);
                const int nibble_subtract = (b6 && !xor1 && active && !xor2);

                wave_address = b7
                    ? ((wave_address - (nibble_add - nibble_subtract)) & 0xfffff)
                    : ((wave_address + (nibble_add - nibble_subtract)) & 0xfffff);

                int newnibble = PCM_ReadROM((hiaddr << 20) | wave_address);
                const int newnibble_sel = address_b4 ^ ( (b6 || !nibble_cmp1) && okey );
                newnibble = newnibble_sel ? ((newnibble >> 4) & 15) : (newnibble & 15);

                int sub_phase    = (ram2[8] & 0x3fff);
                const int step    = pcm.ram2[ ram2[7] & 31 ][0];
                const int interp_ratio = (sub_phase >> 7) & 127;
                sub_phase       += step;
                const int sub_phase_of  = (sub_phase >> 14) & 7;

                if (pcm.nfs) {
                    ram2[8] = (ram2[8] & ~0x3fff) | (sub_phase & 0x3fff);
                }

                // address 0
                int address_cnt = address;
                const int samp0 = (int8_t)PCM_ReadROM((hiaddr << 20) | address_cnt);

                bool nibble_cmp2 = ((address & 0xffff0) == (address_cnt & 0xffff0));
                bool address_cmp = (((b15 ? address_loop : address_end) & 0xfffff) == (address_cnt & 0xfffff));

                int  next_address = address_cnt;
                bool usenew       = !nibble_cmp2;
                bool next_b15     = b15;

                int address_cnt2 = (kon || (!b6 && address_cmp)) ? ((!b6 && address_cmp) ? address_loop : address_cnt) : address_cnt;
                int address_add  = ((!address_cmp && b6 && !b15) || (!address_cmp && !b6)) ? 1 : 0;
                int address_sub  = (!address_cmp && b6 && b15) ? 1 : 0;
                address_cnt = b7 ? ((address_cnt2 - (address_add - address_sub)) & 0xfffff)
                                 : ((address_cnt2 + (address_add - address_sub)) & 0xfffff);
                b15 = b6 && (b15 ^ address_cmp);

                const int samp1 = (int8_t)PCM_ReadROM((hiaddr << 20) | address_cnt);

                bool nibble_cmp3 = ((address & 0xffff0) == (address_cnt & 0xffff0));
                address_cmp = (((b15 ? address_loop : address_end) & 0xfffff) == (address_cnt & 0xfffff));
                if (sub_phase_of >= 1) { next_address = address_cnt; usenew = !nibble_cmp3; next_b15 = b15; }

                address_cnt2 = (kon || (!b6 && address_cmp)) ? ((!b6 && address_cmp) ? address_loop : address_cnt) : address_cnt;
                address_add  = ((!address_cmp && b6 && !b15) || (!address_cmp && !b6)) ? 1 : 0;
                address_sub  = (!address_cmp && b6 && b15) ? 1 : 0;
                address_cnt  = b7 ? ((address_cnt2 - (address_add - address_sub)) & 0xfffff)
                                  : ((address_cnt2 + (address_add - address_sub)) & 0xfffff);
                b15 = b6 && (b15 ^ address_cmp);

                const int samp2 = (int8_t)PCM_ReadROM((hiaddr << 20) | address_cnt);

                bool nibble_cmp4 = ((address & 0xffff0) == (address_cnt & 0xffff0));
                address_cmp = (((b15 ? address_loop : address_end) & 0xfffff) == (address_cnt & 0xfffff));
                if (sub_phase_of >= 2) { next_address = address_cnt; usenew = !nibble_cmp4; next_b15 = b15; }

                address_cnt2 = (kon || (!b6 && address_cmp)) ? ((!b6 && address_cmp) ? address_loop : address_cnt) : address_cnt;
                address_add  = ((!address_cmp && b6 && !b15) || (!address_cmp && !b6)) ? 1 : 0;
                address_sub  = (!address_cmp && b6 && b15) ? 1 : 0;
                address_cnt  = b7 ? ((address_cnt2 - (address_add - address_sub)) & 0xfffff)
                                  : ((address_cnt2 + (address_add - address_sub)) & 0xfffff);
                // b15 по исходнику на этом шаге не обновляется

                const int samp3 = (int8_t)PCM_ReadROM((hiaddr << 20) | address_cnt);
                bool nibble_cmp5 = ((address & 0xffff0) == (address_cnt & 0xffff0));
                address_cmp = (((b15 ? address_loop : address_end) & 0xfffff) == (address_cnt & 0xfffff));
                if (sub_phase_of >= 3) { next_address = address_cnt; usenew = !nibble_cmp5; next_b15 = b15; }

                // ещё один шаг адреса
                address_cnt2 = (kon || (!b6 && address_cmp)) ? ((!b6 && address_cmp) ? address_loop : address_cnt) : address_cnt;
                address_add  = ((!address_cmp && b6 && !b15) || (!address_cmp && !b6)) ? 1 : 0;
                address_sub  = (!address_cmp && b6 && b15) ? 1 : 0;
                address_cnt  = b7 ? ((address_cnt2 - (address_add - address_sub)) & 0xfffff)
                                  : ((address_cnt2 + (address_add - address_sub)) & 0xfffff);

                bool nibble_cmp6 = ((address & 0xffff0) == (address_cnt & 0xffff0));
                if (sub_phase_of >= 4) { next_address = address_cnt; usenew = !nibble_cmp6;  }

                if (active && pcm.nfs) ram1[4] = next_address;
                if (pcm.nfs)           ram2[8] = (ram2[8] & ~0x8000) | (int(next_b15) << 15);

                // ---- DPCM
                int reference = ram1[5];

                int preshift = (samp0 << 10);
                int select_nibble = nibble_cmp2 ? old_nib : newnibble;
                int shift = (10 - select_nibble) & 15;
                int shifted = (preshift << 1) >> shift;
                if (sub_phase_of >= 1) reference = addclip20(reference, (shifted >> 1), (shifted & 1));

                preshift = (samp1 << 10);
                select_nibble = nibble_cmp3 ? old_nib : newnibble;
                shift = (10 - select_nibble) & 15;
                shifted = (preshift << 1) >> shift;
                if (sub_phase_of >= 2) reference = addclip20(reference, (shifted >> 1), (shifted & 1));

                preshift = (samp2 << 10);
                select_nibble = nibble_cmp4 ? old_nib : newnibble;
                shift = (10 - select_nibble) & 15;
                shifted = (preshift << 1) >> shift;
                if (sub_phase_of >= 3) reference = addclip20(reference, (shifted >> 1), (shifted & 1));

                preshift = (samp3 << 10);
                select_nibble = nibble_cmp5 ? old_nib : newnibble;
                shift = (10 - select_nibble) & 15;
                shifted = (preshift << 1) >> shift;
                if (sub_phase_of >= 4) reference = addclip20(reference, (shifted >> 1), (shifted & 1));

                // ---- Интерполяция + IIR (оставил «hack 32-bit math», как у тебя в ветке else)
                int test = ram1[5];

                int step0 = (multi(interp_lut[0][interp_ratio] << 6, samp0) >> 8);
                select_nibble = nibble_cmp2 ? old_nib : newnibble;
                shift = (10 - select_nibble) & 15;
                step0 = (step0 << 1) >> shift;
                test  = addclip20(test, step0 >> 1, step0 & 1);

                int step1 = (multi(interp_lut[1][interp_ratio] << 6, samp1) >> 8);
                select_nibble = nibble_cmp3 ? old_nib : newnibble;
                shift = (10 - select_nibble) & 15;
                step1 = (step1 << 1) >> shift;
                test  = addclip20(test, step1 >> 1, step1 & 1);

                int step2 = (multi(interp_lut[2][interp_ratio] << 6, samp2) >> 8);
                select_nibble = nibble_cmp4 ? old_nib : newnibble;
                shift = (10 - select_nibble) & 15;
                step2 = (step2 << 1) >> shift;

                int reg1   = ram1[1];
                int reg3   = ram1[3];
                const int reg2_6 = (ram2[6] >> 8) & 127;

                test = addclip20(test, step2 >> 1, step2 & 1);

                const int filter = ram2[11];

                // «hack: use 32-bit math to avoid overflow»
                {
                    const int mult1 = reg1 * (int8_t)(filter >> 8);
                    const int mult2 = reg1 * (int8_t)((filter >> 1) & 127);
                    const int mult3 = reg1 * (int8_t)reg2_6;

                    int v2    = reg3 + (mult1 >> 6) + ((mult1 >> 5) & 1);
                    int v1    = v2   + (mult2 >> 13) + ((mult2 >> 12) & 1);
                    int subvar= v1   + (mult3 >> 6)  + ((mult3 >> 5) & 1);

                    // clamp
                    auto Clamp = [](int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); };
                    ram1[3] = Clamp(v1, -0x80000, 0x7ffff);

                    int tests = (test << 12) >> 12;
                    int v3    = tests - subvar;

                    const int mult4 = v3 * (int8_t)(filter >> 8);
                    const int mult5 = v3 * (int8_t)((filter >> 1) & 127);
                    int v4    = reg1 + (mult4 >> 6)  + ((mult4 >> 5) & 1);
                    int v5    = v4   + (mult5 >> 13) + ((mult5 >> 12) & 1);

                    ram1[1] = Clamp(v5, -0x80000, 0x7ffff);

                    // сохранить reference
                    ram1[5] = reference;

                    // IRQ
                    if (active && (ram2[6] & 1) && !(ram2[8] & 0x4000) && !pcm.irq_assert && irq_flag)
                    {
                        if (pcm.nfs) ram2[8] |= 0x4000;
                        pcm.irq_assert = 1;
                        pcm.irq_channel = slot;
                        mcu->MCU_GA_SetGAInt(5, 1);
                    }

                    // огибающие L/R/aux
                    int volmul1 = 0, volmul2 = 0;
                    calc_tv(&pcm, 0, ram2[3],  &ram2[9],  active, &volmul1);
                    calc_tv(&pcm, 1, ram2[4],  &ram2[10], active, &volmul2);
                    calc_tv(&pcm, 2, ram2[5],  &ram2[11], active, nullptr);

                    // выбор источника
                    const int sample = ((ram2[6] & 2) == 0) ? ram1[3] : v3;

                    // громкости (склеил пары мультипликаций)
                    const int multiv1 = multi(sample,    (volmul1 >> 8));
                    const int multiv2 = multi(sample,    ((volmul1 >> 1) & 127));
                    const int sample2 = addclip20(multiv1 >> 6, multiv2 >> 13, ((multiv2 >> 12) | (multiv1 >> 5)) & 1);

                    const int multiv3 = multi(sample2,   (volmul2 >> 8));
                    const int multiv4 = multi(sample2,   ((volmul2 >> 1) & 127));
                    const int sample3 = addclip20(multiv3 >> 6, multiv4 >> 13, ((multiv4 >> 12) | (multiv3 >> 5)) & 1);

                    const int pan = active ? ram2[1] : 0;
                    const int rc  = active ? ram2[2] : 0;

                    const int sampl = multi(sample3, (pan >> 8) & 255);
                    const int sampr = multi(sample3, (pan >> 0) & 255);

                    const int rc0 = (multi(sample3, (rc >> 8) & 255) >> 5); // reverb
                    const int rc1 = (multi(sample3, (rc >> 0) & 255) >> 5); // chorus

                    // write-back state
                    if (key && pcm.nfs)
                    {
                        ram2[7] &= ~0xf020;
                        ram2[7] |= ((usenew || kon) ? newnibble : old_nib) << 12;
                        ram2[7] |= key << 5;
                    }

                    if (!active)
                    {
                        if (pcm.nfs) { ram1[1] = 0; ram1[3] = 0; ram1[5] = 0; }
                        ram2[8] = 0; ram2[9] = 0; ram2[10] = 0;
                    }

                    // ---- сведение RC и основного сигнала
                    const int next_slot = (slot == REG_SLOTS - 1) ? 31 : (slot + 1);
                    switch (next_slot)
                    {
                        case 17: { pcm.ram1[31][1] = addclip20(pcm.ram1[31][1], rcadd[0] >> 1, rcadd[0] & 1);
                                   pcm.rcsum[1]    = addclip20(pcm.rcsum[1],    rcadd2[0] >> 1, rcadd2[0] & 1); break; }
                        case 18: { pcm.ram1[31][3] = addclip20(pcm.ram1[31][3], rcadd[1] >> 1, rcadd[1] & 1);
                                   pcm.rcsum[1]    = addclip20(pcm.rcsum[1],    rcadd2[1] >> 1, rcadd2[1] & 1); break; }
                        case 21: { pcm.ram1[31][1] = addclip20(pcm.ram1[31][1], rcadd[2] >> 1, rcadd[2] & 1);
                                   pcm.rcsum[0]    = addclip20(pcm.rcsum[0],    rcadd2[2] >> 1, rcadd2[2] & 1); break; }
                        case 22: { pcm.ram1[31][3] = addclip20(pcm.ram1[31][3], rcadd[3] >> 1, rcadd[3] & 1);
                                   pcm.rcsum[1]    = addclip20(pcm.rcsum[1],    rcadd2[3] >> 1, rcadd2[3] & 1); break; }
                        case 23: { pcm.ram1[31][1] = addclip20(pcm.ram1[31][1], rcadd[4] >> 1, rcadd[4] & 1);
                                   pcm.rcsum[0]    = addclip20(pcm.rcsum[0],    rcadd2[4] >> 1, rcadd2[4] & 1); break; }
                        case 31: { pcm.ram1[31][3] = addclip20(pcm.ram1[31][3], rcadd[5] >> 1, rcadd[5] & 1);
                                   pcm.rcsum[1]    = addclip20(pcm.rcsum[1],    rcadd2[5] >> 1, rcadd2[5] & 1); break; }
                        default: break;
                    }

                    pcm.rcsum[0] = addclip20(pcm.rcsum[0], rc0 >> 1, rc0 & 1);
                    pcm.rcsum[1] = addclip20(pcm.rcsum[1], rc1 >> 1, rc1 & 1);

                    const int suml = addclip20(pcm.ram1[31][1], (sampl >> 6), ((sampl >> 5) & 1));
                    const int sumr = addclip20(pcm.ram1[31][3], (sampr >> 6), ((sampr >> 5) & 1));

                    if (slot != REG_SLOTS - 1) {
                        pcm.ram1[31][1] = suml;
                        pcm.ram1[31][3] = sumr;
                    } else {
                        pcm.accum_l = suml;
                        pcm.accum_r = sumr;
                    }
                } // IIR block
            } // slots loop

            if (pcm.nfs) {
                pcm.ram2[31][7] |= 0x20;
            }

            pcm_lock.Release();
            pcm.nfs = 1;

        } // for sample_i

        // увеличить счётчик циклов сразу на целый блок
        pcm.cycles += (uint64_t)samples_this_block * (uint64_t)CYCLES_PER_SAMPLE;
    }
}

