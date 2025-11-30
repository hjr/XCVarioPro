/*
 * SetupMenu.h
 *
 *  Created on: Feb 4, 2018
 *      Author: iltis
 */

#pragma once

#include "setup/MenuEntry.h"

#include <cstdint>
#include <string>

class CharFilter {
public:
    CharFilter( const char *chset );
    bool isValidChar( char c );
    char succChar(char c);
    char predChar(char c);
private:
    struct {
        uint16_t upper:1;
        uint16_t lower:1;
        uint16_t digit:1;
        uint16_t special:1;
        uint16_t whitespace:1;
    } _flags;
};

class SetupMenuChar : public MenuEntry {
public:
    SetupMenuChar() = delete;
    SetupMenuChar(const char *title, const char *chset, int mlen, e_restart_mode_t restart = RST_NONE, int (*action)(SetupMenuChar *p) = nullptr, char *achar = 0);
    virtual ~SetupMenuChar() = default;
    void display(int mode = 0) override;
    void rot(int count);
    void press() override;
    void longPress() override;
    const char *value() const override;

    int valSize() { return _value.size(); };

private:
    CharFilter _chfilter;
    int16_t _m_len = 0; // max length of the char array
    bool _mode = false; // false: select char index, true: edit character
    // mutable int _select = 0;
    // int _select_save = 0;
    int16_t _char_index = 0; // position of character to be altered
    std::string _value; // current value as string
    // std::vector<const char *> _values;
    int (*_action)(SetupMenuChar *p);
    // char *_mychar = nullptr;
    bool _dirty = false;
};
