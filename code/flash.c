/* Upright-only parameter storage. New magic prevents interpreting old
 * image/PID records as this compact schema. Pages 0..14 only; marker last.
 * Save is still permitted only with control stopped by menu.c.
 */
#include "flash.h"
uint8 my_flash_check(uint32 sector_num, uint32 page_num)
{
    zf_assert(QCAR_EEPROM_SECTOR_NUM > sector_num);
    zf_assert(QCAR_EEPROM_SECTOR_PAGE_NUM > page_num);
    uint32 sector_addr = IfxFlash_dFlashTableEepLog[sector_num].start;
    uint32 page_addr = sector_addr + page_num * IFXFLASH_DFLASH_PAGE_LENGTH;

    uint32 temp = *(uint32 *)page_addr;
    if(0 !=temp) return 1;
    return 0;


}

//-------------------------------------------------------------------------------------------------------------------
// 擦除指定逻辑扇区。
// sector_num：需要写入的逻辑扇区编号。
//  @return     void
//  @since      v1.0
//  Sample usage:               eeprom_erase_sector(0);
//-------------------------------------------------------------------------------------------------------------------
void eeprom_erase_sector(uint32 sector_num)
{
    zf_assert(QCAR_EEPROM_SECTOR_NUM > sector_num);

    uint32 flash = 0;
    uint16 end_init_sfty_pw;
    uint32 sector_addr = IfxFlash_dFlashTableEepLog[sector_num].start;

    end_init_sfty_pw = IfxScuWdt_getSafetyWatchdogPassword();

    IfxScuWdt_clearSafetyEndinit(end_init_sfty_pw);
    IfxFlash_eraseSector(sector_addr);
    IfxScuWdt_setSafetyEndinit(end_init_sfty_pw);

    IfxFlash_waitUnbusy(flash, IfxFlash_FlashType_D0);

}

//-------------------------------------------------------------------------------------------------------------------
// 编程指定 Flash 物理页。
// sector_num：需要写入的逻辑扇区编号。
// page_num：扇区内需要写入的物理页编号。
// buf：待写入的 uint32 数据地址。
//  @return     void
//  @since      v1.0
//  Sample usage:               eeprom_page_program(0,0, &buf);
//-------------------------------------------------------------------------------------------------------------------
void eeprom_page_program(uint32 sector_num, uint32 page_num, uint32 *buf)
{
    zf_assert(QCAR_EEPROM_SECTOR_NUM>sector_num);
    zf_assert(QCAR_EEPROM_SECTOR_PAGE_NUM>page_num);

    uint32 errors = 0;
    uint32 flash = 0;
    uint16 end_init_sfty_pw;
    uint32 sector_addr = IfxFlash_dFlashTableEepLog[sector_num].start;

    end_init_sfty_pw = IfxScuWdt_getSafetyWatchdogPassword();

    uint32 page_addr = sector_addr + page_num * IFXFLASH_DFLASH_PAGE_LENGTH;
    errors = IfxFlash_enterPageMode(page_addr);
    zf_assert(0 == errors);

    IfxFlash_waitUnbusy(flash, IfxFlash_FlashType_D0);

    IfxFlash_loadPage(page_addr, buf[0], 0);

    IfxScuWdt_clearSafetyEndinit(end_init_sfty_pw);
    IfxFlash_writePage(page_addr);
    IfxScuWdt_setSafetyEndinit(end_init_sfty_pw);

    IfxFlash_waitUnbusy(flash, IfxFlash_FlashType_D0);

}


#define BALANCE_STORAGE_MAGIC 0x4F574232u
#define BALANCE_PARAMETER_COUNT 12u
static float *const values[BALANCE_PARAMETER_COUNT]={
    &roll_rate_test_kp,&roll_rate_test_limit,&roll_rate_test_sign,
    &pitch_balance_kp,&pitch_balance_rate_limit,
    &ground_rate_test_kp,&ground_rate_test_limit,&ground_rate_test_sign,
    &ground_angle_kp,&ground_angle_rate_limit,
    &mono_params.balance_angle_x,&mono_params.balance_angle_y
};
static uint32 checksum_word(uint32 crc,uint32 word) {return (crc*16777619u)^word;}
static int stored_valid(const float v[BALANCE_PARAMETER_COUNT],uint32 ramp)
{
    unsigned i;
    for(i=0;i<BALANCE_PARAMETER_COUNT;++i) if(!isfinite(v[i])) return 0;
    return v[0]>=0 && v[0]<=500 && v[1]>=0 && v[1]<=4000 && (v[2]==1 || v[2]==-1) &&
        v[3]>=0 && v[3]<=100 && v[4]>0 && v[4]<=100 &&
        v[5]>=0 && v[5]<=500 && v[6]>=0 && v[6]<=4000 && (v[7]==1 || v[7]==-1) &&
        v[8]>=0 && v[8]<=100 && v[9]>0 && v[9]<=100 &&
        fabsf(v[10])<=20 && fabsf(v[11])<=20 && ramp<=5000;
}
int write_flash(void)
{
    float snapshot[BALANCE_PARAMETER_COUNT];
    uint32 words[BALANCE_PARAMETER_COUNT],crc=BALANCE_STORAGE_MAGIC,magic=BALANCE_STORAGE_MAGIC;
    uint32 ramp,irq; unsigned i;
    irq=interrupt_global_disable();
    for(i=0;i<BALANCE_PARAMETER_COUNT;++i) snapshot[i]=*values[i];
    ramp=control_startup_ramp_ms;
    interrupt_global_enable(irq);
    if(!stored_valid(snapshot,ramp)) return 0;
    memcpy(words,snapshot,sizeof(words));
    eeprom_erase_sector(0);
    for(i=0;i<BALANCE_PARAMETER_COUNT;++i) {
        eeprom_page_program(0,i+1,&words[i]);crc=checksum_word(crc,words[i]);
    }
    eeprom_page_program(0,13,&ramp);crc=checksum_word(crc,ramp);
    eeprom_page_program(0,14,&crc);
    eeprom_page_program(0,0,&magic);
    IfxFlash_resetToRead(0);
    if(my_flash_read(0,0,uint32)!=magic || my_flash_read(0,14,uint32)!=crc) return 0;
    for(i=0;i<BALANCE_PARAMETER_COUNT;++i) if(my_flash_read(0,i+1,uint32)!=words[i])return 0;
    return my_flash_read(0,13,uint32)==ramp;
}
void get_flash(void)
{
    balance_control_defaults();
#if QCAR_BOOT_LOAD_FLASH
    float snapshot[BALANCE_PARAMETER_COUNT];
    uint32 words[BALANCE_PARAMETER_COUNT],crc=BALANCE_STORAGE_MAGIC,ramp;unsigned i;
    IfxFlash_resetToRead(0);
    if(my_flash_read(0,0,uint32)!=BALANCE_STORAGE_MAGIC) return;
    for(i=0;i<BALANCE_PARAMETER_COUNT;++i) {words[i]=my_flash_read(0,i+1,uint32);crc=checksum_word(crc,words[i]);}
    ramp=my_flash_read(0,13,uint32);crc=checksum_word(crc,ramp);
    if(crc!=my_flash_read(0,14,uint32))return;
    memcpy(snapshot,words,sizeof(snapshot));
    if(!stored_valid(snapshot,ramp))return;
    for(i=0;i<BALANCE_PARAMETER_COUNT;++i)*values[i]=snapshot[i];
    control_startup_ramp_ms=ramp;
#endif
}
