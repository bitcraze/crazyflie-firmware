/*----------------------------------------------------------------------/
/ Low level disk I/O module function checker
/-----------------------------------------------------------------------/
/ WARNING: The data on the target drive will be lost!
*/

#define DEBUG_MODULE ""

#include <stdio.h>
#include <string.h>
#include "ff.h"
#include "diskio.h"
#include "debug.h"

static
DWORD pn (
    DWORD pns
)
{
    static DWORD lfsr;
    UINT n;


    if (pns) {
        lfsr = pns;
        for (n = 0; n < 32; n++) pn(0);
    }
    if (lfsr & 1) {
        lfsr >>= 1;
        lfsr ^= 0x80200003;
    } else {
        lfsr >>= 1;
    }
    return lfsr;
}


int test_diskio (
    BYTE pdrv,      /* Physical drive number to be checked (all data on the drive will be lost) */
    UINT ncyc,      /* Number of test cycles */
    DWORD* buff,    /* Pointer to the working buffer */
    UINT sz_buff    /* Size of the working buffer in unit of byte */
)
{
    UINT n, cc, ns;
    DWORD sz_drv, lba, lba2, sz_eblk, pns = 1;
    WORD sz_sect;
    BYTE *pbuff = (BYTE*)buff;
    DSTATUS ds;
    DRESULT dr;


    DEBUG_PRINT("test_diskio(%u, %u, 0x%08X, 0x%08X)\n", pdrv, ncyc, (UINT)buff, sz_buff);

    if (sz_buff < _MAX_SS + 4) {
        DEBUG_PRINT("Insufficient work area to test.\n");
        return 1;
    }

    for (cc = 1; cc <= ncyc; cc++) {
        DEBUG_PRINT("**** Test cycle %u of %u start ****\n", cc, ncyc);

        /* Initialization */
        DEBUG_PRINT(" disk_initalize(%u)", pdrv);
        ds = disk_initialize(pdrv);
        if (ds & STA_NOINIT) {
            DEBUG_PRINT("[FAIL].\n");
            return 2;
        } else {
            DEBUG_PRINT("[OK].\n");
        }

        /* Get drive size */
        DEBUG_PRINT("**** Get drive size ****\n");
        DEBUG_PRINT(" disk_ioctl(%u, GET_SECTOR_COUNT, 0x%08X)", pdrv, (UINT)&sz_drv);
        sz_drv = 0;
        dr = disk_ioctl(pdrv, GET_SECTOR_COUNT, &sz_drv);
        if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
        } else {
            DEBUG_PRINT("[FAIL].\n");
            return 3;
        }
        if (sz_drv < 128) {
            DEBUG_PRINT("Failed: Insufficient drive size to test.\n");
            return 4;
        }
        DEBUG_PRINT(" Number of sectors on the drive %u is %lu.\n", pdrv, sz_drv);

#if _MAX_SS != _MIN_SS
        /* Get sector size */
        DEBUG_PRINT("**** Get sector size ****\n");
        DEBUG_PRINT(" disk_ioctl(%u, GET_SECTOR_SIZE, 0x%X)", pdrv, (UINT)&sz_sect);
        sz_sect = 0;
        dr = disk_ioctl(pdrv, GET_SECTOR_SIZE, &sz_sect);
        if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
        } else {
            DEBUG_PRINT("[FAIL].\n");
            return 5;
        }
        DEBUG_PRINT(" Size of sector is %u bytes.\n", sz_sect);
#else
        sz_sect = _MAX_SS;
#endif

        /* Get erase block size */
        DEBUG_PRINT("**** Get block size ****\n");
        DEBUG_PRINT(" disk_ioctl(%u, GET_BLOCK_SIZE, 0x%X)", pdrv, (UINT)&sz_eblk);
        sz_eblk = 0;
        dr = disk_ioctl(pdrv, GET_BLOCK_SIZE, &sz_eblk);
        if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
        } else {
            DEBUG_PRINT("[FAIL].\n");
        }
        if (dr == RES_OK || sz_eblk >= 2) {
            DEBUG_PRINT(" Size of the erase block is %lu sectors.\n", sz_eblk);
        } else {
            DEBUG_PRINT(" Size of the erase block is unknown.\n");
        }

        /* Single sector write test */
        DEBUG_PRINT("**** Single sector write test 1 ****\n");
        lba = 0;
        for (n = 0, pn(pns); n < sz_sect; n++) pbuff[n] = (BYTE)pn(0);
        DEBUG_PRINT(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT)pbuff, lba);
        dr = disk_write(pdrv, pbuff, lba, 1);
        if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
        } else {
            DEBUG_PRINT("[FAIL].\n");
            return 6;
        }
        DEBUG_PRINT(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
        dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
        if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
        } else {
            DEBUG_PRINT("[FAIL].\n");
            return 7;
        }
        memset(pbuff, 0, sz_sect);
        DEBUG_PRINT(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT)pbuff, lba);
        dr = disk_read(pdrv, pbuff, lba, 1);
        if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
        } else {
            DEBUG_PRINT("[FAIL].\n");
            return 8;
        }
        for (n = 0, pn(pns); n < sz_sect && pbuff[n] == (BYTE)pn(0); n++) ;
        if (n == sz_sect) {
            DEBUG_PRINT(" Data matched.\n");
        } else {
            DEBUG_PRINT("Failed: Read data differs from the data written.\n");
            return 10;
        }
        pns++;

        /* Multiple sector write test */
        DEBUG_PRINT("**** Multiple sector write test ****\n");
        lba = 1; ns = sz_buff / sz_sect;
        if (ns > 4) ns = 4;
        for (n = 0, pn(pns); n < (UINT)(sz_sect * ns); n++) pbuff[n] = (BYTE)pn(0);
        DEBUG_PRINT(" disk_write(%u, 0x%X, %lu, %u)", pdrv, (UINT)pbuff, lba, ns);
        dr = disk_write(pdrv, pbuff, lba, ns);
        if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
        } else {
            DEBUG_PRINT("[FAIL].\n");
            return 11;
        }
        DEBUG_PRINT(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
        dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
        if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
        } else {
            DEBUG_PRINT("[FAIL].\n");
            return 12;
        }
        memset(pbuff, 0, sz_sect * ns);
        DEBUG_PRINT(" disk_read(%u, 0x%X, %lu, %u)", pdrv, (UINT)pbuff, lba, ns);
        dr = disk_read(pdrv, pbuff, lba, ns);
        if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
        } else {
            DEBUG_PRINT("[FAIL].\n");
            return 13;
        }
        for (n = 0, pn(pns); n < (UINT)(sz_sect * ns) && pbuff[n] == (BYTE)pn(0); n++) ;
        if (n == (UINT)(sz_sect * ns)) {
            DEBUG_PRINT(" Data matched.\n");
        } else {
            DEBUG_PRINT("Failed: Read data differs from the data written.\n");
            return 14;
        }
        pns++;

        /* Single sector write test (misaligned memory address) */
        DEBUG_PRINT("**** Single sector write test 2 ****\n");
        lba = 5;
        for (n = 0, pn(pns); n < sz_sect; n++) pbuff[n+3] = (BYTE)pn(0);
        DEBUG_PRINT(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT)(pbuff+3), lba);
        dr = disk_write(pdrv, pbuff+3, lba, 1);
        if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
        } else {
            DEBUG_PRINT("[FAIL].\n");
            return 15;
        }
        DEBUG_PRINT(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
        dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
        if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
        } else {
            DEBUG_PRINT("[FAIL].\n");
            return 16;
        }
        memset(pbuff+5, 0, sz_sect);
        DEBUG_PRINT(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT)(pbuff+5), lba);
        dr = disk_read(pdrv, pbuff+5, lba, 1);
        if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
        } else {
            DEBUG_PRINT("[FAIL].\n");
            return 17;
        }
        for (n = 0, pn(pns); n < sz_sect && pbuff[n+5] == (BYTE)pn(0); n++) ;
        if (n == sz_sect) {
            DEBUG_PRINT(" Data matched.\n");
        } else {
            DEBUG_PRINT("Failed: Read data differs from the data written.\n");
            return 18;
        }
        pns++;

        /* 4GB barrier test */
        DEBUG_PRINT("**** 4GB barrier test ****\n");
        if (sz_drv >= 128 + 0x80000000 / (sz_sect / 2)) {
            lba = 6; lba2 = lba + 0x80000000 / (sz_sect / 2);
            for (n = 0, pn(pns); n < (UINT)(sz_sect * 2); n++) pbuff[n] = (BYTE)pn(0);
            DEBUG_PRINT(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT)pbuff, lba);
            dr = disk_write(pdrv, pbuff, lba, 1);
            if (dr == RES_OK) {
                DEBUG_PRINT("[OK].\n");
            } else {
                DEBUG_PRINT("[FAIL].\n");
                return 19;
            }
            DEBUG_PRINT(" disk_write(%u, 0x%X, %lu, 1)", pdrv, (UINT)(pbuff+sz_sect), lba2);
            dr = disk_write(pdrv, pbuff+sz_sect, lba2, 1);
            if (dr == RES_OK) {
                DEBUG_PRINT("[OK].\n");
            } else {
                DEBUG_PRINT("[FAIL].\n");
                return 20;
            }
            DEBUG_PRINT(" disk_ioctl(%u, CTRL_SYNC, NULL)", pdrv);
            dr = disk_ioctl(pdrv, CTRL_SYNC, 0);
            if (dr == RES_OK) {
            DEBUG_PRINT("[OK].\n");
            } else {
                DEBUG_PRINT("[FAIL].\n");
                return 21;
            }
            memset(pbuff, 0, sz_sect * 2);
            DEBUG_PRINT(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT)pbuff, lba);
            dr = disk_read(pdrv, pbuff, lba, 1);
            if (dr == RES_OK) {
                DEBUG_PRINT("[OK].\n");
            } else {
                DEBUG_PRINT("[FAIL].\n");
                return 22;
            }
            DEBUG_PRINT(" disk_read(%u, 0x%X, %lu, 1)", pdrv, (UINT)(pbuff+sz_sect), lba2);
            dr = disk_read(pdrv, pbuff+sz_sect, lba2, 1);
            if (dr == RES_OK) {
                DEBUG_PRINT("[OK].\n");
            } else {
                DEBUG_PRINT("[FAIL].\n");
                return 23;
            }
            for (n = 0, pn(pns); pbuff[n] == (BYTE)pn(0) && n < (UINT)(sz_sect * 2); n++) ;
            if (n == (UINT)(sz_sect * 2)) {
                DEBUG_PRINT(" Data matched.\n");
            } else {
                DEBUG_PRINT("Failed: Read data differs from the data written.\n");
                return 24;
            }
        } else {
            DEBUG_PRINT(" Test skipped.\n");
        }
        pns++;

        DEBUG_PRINT("**** Test cycle %u of %u completed ****\n\n", cc, ncyc);
    }

    return 0;
}

