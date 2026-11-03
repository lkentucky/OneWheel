#include "Controller/Planner/path_store.h"
#include <string.h>

#if !defined(PATH_HOST_TEST) || defined(PATH_STORE_TEST)
#ifdef PATH_STORE_TEST
#include "path_store_stubs.h"
#else
#include "Controller/control.h"
#include "zf_driver_flash.h"
#endif
#pragma section all "cpu0_dsram"
typedef char path_page_fits[(PATH_RECORD_WORDS <= EEPROM_PAGE_LENGTH) ? 1 : -1];
#endif

typedef char path_point_wire_size[(sizeof(path_point_t) == 12u) ? 1 : -1];

#define PATH_MAGIC 0x50415448u
#define PATH_FORMAT 1u

static uint32_t crc_words(const uint32_t *w, unsigned count)
{
    uint32_t crc = 0xffffffffu;
    unsigned i, b;
    for (i = 0; i < count; i++)
    {
        uint32_t v = w[i];
        for (b = 0; b < 32u; b++)
        {
            uint32_t bit = (crc ^ v) & 1u;
            crc >>= 1;
            if (bit) crc ^= 0xedb88320u;
            v >>= 1;
        }
    }
    return ~crc;
}

unsigned path_record_encode(uint32_t *w, const path_point_t *p, uint16_t count, uint32_t gen)
{
    unsigned n = 4u + (unsigned)count * 3u;
    if (!w || (count != 0u && !path_nav_validate(p, count))) return 0;
    w[0] = PATH_MAGIC; w[1] = PATH_FORMAT; w[2] = gen; w[3] = count;
    if (count) memcpy(w+4, p, (size_t)count * sizeof(p[0]));
    w[n] = crc_words(w, n);
    return n+1u;
}

int path_record_decode(const uint32_t *w, path_point_t *p, uint16_t *count, uint32_t *gen)
{
    unsigned n;
    uint16_t c;
    if (!w || !p || !count || !gen || w[0] != PATH_MAGIC || w[1] != PATH_FORMAT ||
        w[3] > PATH_MAX_POINTS || w[3] == 1u) return 0;
    c = (uint16_t)w[3];
    n = 4u + (unsigned)c * 3u;
    if (w[n] != crc_words(w, n)) return 0;
    if (c) memcpy(p, w+4, (size_t)c * sizeof(p[0]));
    if (c && !path_nav_validate(p, c)) return 0;
    *count = c; *gen = w[2];
    return 1;
}

#if !defined(PATH_HOST_TEST) || defined(PATH_STORE_TEST)
static uint32_t s_record[PATH_RECORD_WORDS];
static uint32_t s_verify[PATH_RECORD_WORDS];
static path_point_t s_candidate[PATH_MAX_POINTS];

static int read_slot(unsigned page, uint16_t *count, uint32_t *gen)
{
    if (!flash_check(0, page)) return 0;
    flash_read_page(0, page, (uint32 *)s_record, (uint16)PATH_RECORD_WORDS);
    return path_record_decode(s_record, s_candidate, count, gen);
}

static unsigned latest_slot(uint32_t *generation)
{
    uint16_t count;
    uint32_t a = 0, b = 0;
    int va = read_slot(PATH_FLASH_PAGE_A, &count, &a);
    int vb = read_slot(PATH_FLASH_PAGE_B, &count, &b);
    if (vb && (!va || (int32_t)(b-a) > 0)) { *generation = b; return PATH_FLASH_PAGE_B; }
    if (va) { *generation = a; return PATH_FLASH_PAGE_A; }
    *generation = 0;
    return 0;
}

static int load_latest(void)
{
    uint32_t gen;
    uint16_t count;
    unsigned page = latest_slot(&gen);
    if (!page || !read_slot(page, &count, &gen)) return 0;
    if (!count) { path_nav_clear(); path_nav_saved(); return 1; }
    return path_nav_import(s_candidate, count);
}

void path_store_init(void)
{
    path_nav_init();
    (void)load_latest();
}

int path_store_load(void)
{
    if (!control_path_storage_ready()) return 0;
    return load_latest();
}

int path_store_save(void)
{
    uint32_t gen;
    uint16_t i;
    path_status_t status;
    unsigned latest, target, words;
    if (!control_path_storage_ready()) return 0;
    path_nav_status(&status);
    if (status.count && !status.valid) return 0;
    latest = latest_slot(&gen);
    target = latest == PATH_FLASH_PAGE_A ? PATH_FLASH_PAGE_B : PATH_FLASH_PAGE_A;
    for (i = 0; i < status.count; i++)
        if (!path_nav_point(i, &s_candidate[i])) return 0;
    words = path_record_encode(s_record, s_candidate, status.count, gen+1u);
    if (!words) return 0;
    /* Explicit erase avoids dependence on the vendor's global erase flag. */
    flash_erase_page(0, target);
    flash_write_page(0, target, (const uint32 *)s_record, (uint16)words);
    flash_read_page(0, target, (uint32 *)s_verify, (uint16)words);
    if (memcmp(s_record, s_verify, words*sizeof(s_record[0])) != 0) return 0;
    path_nav_saved();
    return 1;
}
#pragma section all restore
#endif
