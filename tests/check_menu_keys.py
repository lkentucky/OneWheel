"""Compile and exercise the actual Qcar Key_* / Menu_Switch implementation."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'code/menu.c').read_text(encoding='utf-8')
keys = source[source.index('void Key_Up(void)'):source.index('void Menu_Show(void)')]
prefix = r'''
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "menu_tree.h"
typedef int16_t int16;
typedef enum { KEY_1, KEY_2, KEY_3, KEY_4 } key_index_enum;
typedef enum { KEY_RELEASE, KEY_SHORT_PRESS, KEY_LONG_PRESS } key_state_enum;
static key_state_enum states[4];
static Menu_Item *pointer;
static float steps[] = {.01f,.1f,1,10,100};
static uint8_t step_index = 2;
static void Menu_Request_Redraw(void) {}
static bool Menu_Handle_Control_Bool(Menu_Item *item, bool value) { return false; }
static bool Menu_Handle_Control_Uint8(Menu_Item *item, int16 delta) { return false; }
static void ips200_show_string(int x,int y,const char *s) {}
static key_state_enum key_get_state(key_index_enum key) { return states[key]; }
static void key_consume_state(key_index_enum key, key_state_enum event) { assert(states[key] == event); states[key] = KEY_RELEASE; }
'''
main = r'''
static void press(int key) { states[key] = KEY_SHORT_PRESS; Menu_Switch(); }
int main(void) {
    Menu_Item root = {0}; root.kind = MENU_Folder;
    Menu_Tree_Reset();
    Menu_Item *folder = Create_Menu_Folder_dynamic(&root,"F");
    float value = 0; bool toggle = false;
    Create_Menu_File_dynamic(folder,"Value",&value,float_Box);
    Create_Menu_File_dynamic(folder,"Toggle",&toggle,bool_Box);
    All_Folder_Menu_Init(&root); pointer = folder;
    press(KEY_3); assert(pointer == folder->First_Son);
    press(KEY_3); assert(pointer->selected);
    press(KEY_1); assert(value == 1);
    press(KEY_2); assert(value == 0);
    press(KEY_3); assert(step_index == 1 && pointer->selected);
    press(KEY_3); assert(step_index == 0);
    press(KEY_3); assert(step_index == 4);
    press(KEY_4); assert(!pointer->selected);
    press(KEY_2); assert(pointer->kind == bool_Box);
    press(KEY_3); press(KEY_1); assert(toggle);
    press(KEY_2); assert(!toggle);
    press(KEY_4); press(KEY_4); assert(pointer == folder);
    press(KEY_4); assert(pointer == folder);
    press(KEY_3); press(KEY_1); assert(pointer->kind == bool_Box);
    states[KEY_1] = states[KEY_2] = KEY_SHORT_PRESS;
    Menu_Switch(); assert(pointer->kind == float_Box);
    assert(states[KEY_2] == KEY_SHORT_PRESS);
    Menu_Switch(); assert(pointer->kind == bool_Box);
    for (int i=0;i<4;i++) assert(states[i] == KEY_RELEASE);
    int32_t i32=0; uint32_t u32=0; int16_t i16=0; uint16_t u16=0;
    int8_t i8=0; uint8_t u8=0;
    void *values[]={&i32,&u32,&i16,&u16,&i8,&u8};
    Menu_Kind kinds[]={int32_Box,uint32_Box,int16_Box,uint16_Box,int8_Box,uint8_Box};
    Menu_Item item={0}; pointer=&item; step_index=2;
    for(int i=0;i<6;i++) { item.kind=kinds[i]; item.data=values[i]; Key_Plus(); Key_Sub(); }
    assert(i32==0 && u32==0 && i16==0 && u16==0 && i8==0 && u8==0);
    item.kind=float_Box;item.data=&value;item.step_scale=.001f;value=0;
    Key_Plus();assert(value>.00099f && value<.00101f);
    Key_Sub();assert(value<.000001f && value>-.000001f);
    step_index=0;Key_Plus();assert(value>.0000099f && value<.0000101f);
    puts("PASS: updated key mapping, navigation, editing, step cycle, bool, types, priority and clearing");
}
'''
with tempfile.TemporaryDirectory(prefix='qcar-keys-') as folder:
    work = Path(folder)
    test = work / 'keys.c'
    test.write_text(prefix + keys + main, encoding='utf-8')
    exe = work / 'keys.exe'
    subprocess.run(['gcc','-std=c99','-I'+str(root/'code'),str(root/'code/menu_tree.c'),
                    str(test),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
