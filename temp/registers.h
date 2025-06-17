#ifndef REGISTERS_H_
#define REGISTERS_H_

// CTRL1 Register
struct CTRL1_Register
{
    unsigned int Address;  // bit 23-16
    unsigned int AG_SETPT; // bit 15-12
    unsigned int ENPOL;    // bit 11
    unsigned int DIRPOL;   // bit 10
    unsigned int BRKPOL;   // bit 9
    unsigned int SYNRECT;  // bit 8
    unsigned int PWMF;     // bit 7-6
    unsigned int SPDMODE;  // bit 5-4
    unsigned int FGSEL;    // bit 3-2
    unsigned int BRKMOD;   // bit 1
    unsigned int RETRY;    // bit 0
};

// ADVANCE Register
struct ADVANCE_Register
{
    unsigned int Address; // bit 23-16
    /* Reserved */        // bit 15-8
    unsigned int ADVANCE; // bit 7-0
};

// COMCTRL1 Register
struct COMCTRL1_Register
{
    unsigned int Address; // bit 23-16
    unsigned int SPDREVS; // bit 15-8
    unsigned int MINSPD;  // bit 7-0
};

// MOD120 Register
struct MOD120_Register
{
    unsigned int Address; // bit 23-16
    unsigned int BASIC;   // bit 15
    unsigned int SPEEDTH; // bit 14-12
    unsigned int MOD120;  // bit 11-0
};

// DRIVE Register
struct DRIVE_Register
{
    unsigned int Address; // bit 23-16
    unsigned int LRTIME;  // bit 15-14
    unsigned int HALLRST; // bit 13-12
    unsigned int DELAY;   // bit 11
    unsigned int AUTOADV; // bit 10
    unsigned int AUTOGN;  // bit 9
    unsigned int ENSINE;  // bit 8
    unsigned int TDRIVE;  // bit 7-6
    unsigned int DTIME;   // bit 5-3
    unsigned int IDRIVE;  // bit 2-0
};

// SPDGAIN Register
struct SPDGAIN_Register
{
    unsigned int Address; // bit 23-16
    /* Reserved */        // bit 15
    unsigned int INTCLK;  // bit 14-12
    unsigned int SPDGAIN; // bit 11-0
};

// FILK1 Register
struct FILK1_Register
{
    unsigned int Address; // bit 23-16
    unsigned int HALLPOL; // bit 15
    /* Reserved */        // bit 14-13
    unsigned int BYPFILT; // bit 12
    unsigned int FILK1;   // bit 11-0
};

// FILK2 Register
struct FILK2_Register
{
    unsigned int Address; // bit 23-16
    /* Reserved */        // bit 15-12
    unsigned int FILK2;   // bit 11-0
};

// COMPK1 Register
struct COMPK1_Register
{
    unsigned int Address; // bit 23-16
    /* Reserved */        // bit 15-13
    unsigned int BYPCOMP; // bit 12
    unsigned int COMPK1;  // bit 11-0
};

// COMPK2 Register
struct COMPK2_Register
{
    unsigned int Address;  // bit 23-16
    unsigned int AA_SETPT; // bit 15-12
    unsigned int COMPK2;   // bit 11-0
};

// LOOPGN Register
struct LOOPGN_Register
{
    unsigned int Address; // bit 23-16
    unsigned int OCPDEG;  // bit 15-14
    unsigned int OCPTH;   // bit 13-12
    unsigned int OVTH;    // bit 11
    unsigned int VREF_EN; // bit 10
    unsigned int LOOPGN;  // bit 9-0
};

// SPEED Register
struct SPEED_Register
{
    unsigned int Address; // bit 23-16
    /* Reserved */        // bit 15-12
    unsigned int SPEED;   // bit 11-10
};

// FAULT Register
struct FAULT_Register
{
    unsigned int Address; // bit 23-16
    /* Reserved */        // bit 15-7
    unsigned int RLOCK;   // bit 6
    unsigned int VMOV;    // bit 5
    unsigned int CPFAIL;  // bit 4
    unsigned int UVLO;    // bit 3
    unsigned int OTS;     // bit 2
    unsigned int CPOC;    // bit 1
    unsigned int OCP;     // bit 0
};

#endif /* REGISTERS_H_ */