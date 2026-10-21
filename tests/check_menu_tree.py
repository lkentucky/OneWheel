"""Host check of the transplanted menu tree, using GCC."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='qcar-menu-') as directory:
    work = Path(directory)
    test = work / 'check.c'
    test.write_text('''
#include <assert.h>
#include <stdio.h>
#include "menu_tree.h"
int main(void) {
    Menu_Item root = {0};
    root.kind = MENU_Folder;
    All_Folder_Menu_Init(&root);
    Menu_Tree_Reset();
    Menu_Item *folder = Create_Menu_Folder_dynamic(&root, "PID");
    float value = 1;
    Create_Menu_File_dynamic(folder, "Kp", &value, float_Box);
    Create_Menu_File_dynamic(folder, "Ki", &value, float_Box);
    All_Folder_Menu_Init(&root);
    assert(root.First_Son == folder);
    assert(folder->Next_Brother == folder && folder->Last_Brother == folder);
    Menu_Item *first = folder->First_Son;
    assert(first->rank == 1 && first->Next_Brother->rank == 2);
    assert(first->Last_Brother == first->Next_Brother);
    assert(first->Next_Brother->Next_Brother == first);
    All_Folder_Menu_Init(&root);
    assert(first->Next_Brother->Next_Brother == first);
    Menu_Tree_Reset();
    root = (Menu_Item){0};
    root.kind = MENU_Folder;
    for (int i = 0; i < MENU_MAX_SIZE; i++)
        assert(Create_Menu_Folder_dynamic(&root, "item") != NULL);
    assert(Create_Menu_Folder_dynamic(&root, "overflow") == NULL);
    All_Folder_Menu_Init(&root);
    assert(root.sons == MENU_MAX_SIZE);
    puts("PASS: empty/single/nested circular tree, repeat linking, pool limit");
}
''', encoding='utf-8')
    exe = work / 'check.exe'
    subprocess.run(['gcc', '-std=c99', '-I' + str(root / 'code'),
                    str(root / 'code/menu_tree.c'), str(test), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=5)
