
#pragma once

class SetupMenu;
class SetupMenuSelect;
class SetupMenuValFloat;

void audio_menu_create(SetupMenu *top);
int update_range_entry_s(SetupMenuSelect *p);
void free_audio_menu();