/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "setup/SetupMenu.h"
#include "setup/SetupMenuSelect.h"
#include "setup/SetupNG.h"
#include "Colors.h"
#include "AdaptUGC.h"
#include "logdef.h"

#include <cstdint>

extern AdaptUGC *MYUCG;

static int action_select_ap(SetupMenuSelect* p) {
    if (p->getSelect() == 1) {
    }
    p->setSelect(0);
    return 0;
}


void software_menu_create_OTA(SetupMenu *top) {

    // text
    SetupMenuSelect *text = new SetupMenuSelect("* Internet access through", RST_NONE);
    text->lock();
    top->addEntry(text);

    SetupMenuSelect *ap_list = new SetupMenuSelect("Access Point", RST_NONE, action_select_ap, nullptr);
    ap_list->setHelp("Select an access point to connect to the internet");
    ap_list->addEntry("Cancel");
    ap_list->addEntry("Refresh List");
    // for (auto &ap : wifi_scan_results) {
    //     ap_list->addEntry(ap.ssid.c_str());
    // }
    top->addEntry(ap_list);

    SetupMenuSelect* upd = new SetupMenuSelect("Update", RST_IMMEDIATE, nullptr, &software_update);
    upd->setHelp("Update using the internet connection of your smart phone, or upload a binary using the ESP32 webserver.");
    upd->addEntry("Cancel");
    upd->addEntry("Easy Connect");
    upd->addEntry("Webserver");
    top->addEntry(upd);



}


// void free_imu_menu()
// {
//     imu_menu_help.clear();
//     imu_menu_help.shrink_to_fit();
// }
