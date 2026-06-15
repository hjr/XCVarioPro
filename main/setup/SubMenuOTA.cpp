/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "SubMenuOTA.h"

#include "comm/WifiApSta.h"
#include "setup/SetupMenu.h"
#include "setup/SetupMenuSelect.h"
#include "setup/SetupNG.h"
#include "Colors.h"
#include "AdaptUGC.h"
#include "logdef.h"

#include <cstdint>

extern AdaptUGC *MYUCG;

wifi_ap_record_t *wifi_scan_results = nullptr;
constexpr const uint16_t MAX_WIFI_SCAN_RESULTS = 20;

static int action_select_ap(SetupMenuSelect* p) {
    if (p->getValue() == 101) {
        if ( !WIFI ) {
            WifiApSta::createWifiApSta();
        }
        if ( !WIFI->isSTA() ) { 
            ESP_LOGI(FNAME, "Switch to STA mode");
            WIFI->ConfigureIntf(9999);
        }
        if ( wifi_scan_results == nullptr ) {
            wifi_scan_results = new wifi_ap_record_t[MAX_WIFI_SCAN_RESULTS];
        }
        uint16_t num_aps = 0;
        WIFI->scanAPs(wifi_scan_results, num_aps, MAX_WIFI_SCAN_RESULTS);
        ESP_LOGI(FNAME, "Found %d APs", num_aps);
        p->delAllEntries();
        p->addEntry("Select AP", 101);
        for (int i = 0; i < num_aps; ++i) {
            p->addEntry((const char*)wifi_scan_results[i].ssid, i);
            ESP_LOGI(FNAME, "Found AP: %s", wifi_scan_results[i].ssid);
        }
    }
    p->setSelect(0);
    return 0;
}


static int action_check_updates(SetupMenuSelect* p) {
    if (p->getSelect() == 1) {
        // check for updates
    }
    else if ( p->getSelect() == 2) {
        // re-start into webserver mode
        p->reBoot();
    }
    p->setSelect(0);
    return 0;
}

void software_menu_create_OTA(SetupMenu *top) {

    // text
    SetupMenuSelect *text = new SetupMenuSelect(">Internet access<", RST_NONE);
    text->lock();
    top->addEntry(text);

    SetupMenuSelect *ap_list = new SetupMenuSelect("AP", RST_NONE, action_select_ap, nullptr);
    ap_list->setHelp("Select an access point to connect to the internet");
    ap_list->addEntry("Cancel", 100);
    ap_list->addEntry("Select AP", 101);
    // ap_list->addEntry("Refresh List", 101);
    top->addEntry(ap_list);

    SetupMenuSelect* upd = new SetupMenuSelect("Update", RST_NONE, action_check_updates, &software_update);
    upd->setHelp("Update using the internet connection of your smart phone, or upload a binary using the ESP32 webserver.");
    upd->addEntry("Cancel");
    upd->addEntry("Check for Updates");
    upd->addEntry("Webserver");
    top->addEntry(upd);

}


void free_ota_menu()
{
    if ( wifi_scan_results ) {
        delete[] wifi_scan_results;
        wifi_scan_results = nullptr;
    }
}
