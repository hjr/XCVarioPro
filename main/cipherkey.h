#pragma once

#if __has_include("../cipherkey.h.in")
#  include "../cipherkey.h.in"
#else
#  define CIPHER_KEY "1234"
#endif
