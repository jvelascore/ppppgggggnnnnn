/* locale_c.h Standard "C" locale definitions. */
/* Copyright 2009-2010 IAR Systems AB. */
#ifndef _LOCALE_C_H
#define _LOCALE_C_H

#ifndef _SYSTEM_BUILD
  #pragma system_include
#endif

#include <ycheck.h>
#include <yvals.h>

#if _DLIB_WIDE_CHARACTERS
#include <wchar.h>
#endif

_C_STD_BEGIN


_C_LIB_DECL

__ATTRIBUTES int _LocaleC_toupper(int);
__ATTRIBUTES int _LocaleC_tolower(int);

__ATTRIBUTES int _LocaleC_isalpha(int);
__ATTRIBUTES int _LocaleC_iscntrl(int);
__ATTRIBUTES int _LocaleC_islower(int);
__ATTRIBUTES int _LocaleC_ispunct(int);
__ATTRIBUTES int _LocaleC_isspace(int);
__ATTRIBUTES int _LocaleC_isupper(int);

#if _DLIB_WIDE_CHARACTERS

__ATTRIBUTES wint_t _LocaleC_towupper(wint_t);
__ATTRIBUTES wint_t _LocaleC_towlower(wint_t);

__ATTRIBUTES int _LocaleC_iswalpha(wint_t);
__ATTRIBUTES int _LocaleC_iswcntrl(wint_t);
__ATTRIBUTES int _LocaleC_iswlower(wint_t);
__ATTRIBUTES int _LocaleC_iswpunct(wint_t);
__ATTRIBUTES int _LocaleC_iswspace(wint_t);
__ATTRIBUTES int _LocaleC_iswupper(wint_t);
__ATTRIBUTES int _LocaleC_iswdigit(wint_t);
__ATTRIBUTES int _LocaleC_iswxdigit(wint_t);

#endif /* _DLIB_WIDE_CHARACTERS */

_END_C_LIB_DECL

/*
 * Inline definitions.
 */

#ifndef _NO_DEFINITIONS_IN_HEADER_FILES
  /* Note: The first two must precede the functions they are used in. */
  #pragma inline
  int _LocaleC_islower(int _C)
  {
    return (_C>='a' && _C<='z');
  }

  #pragma inline
  int _LocaleC_isupper(int _C)
  {
    return (_C>='A' && _C<='Z');
  }

  #pragma inline
  int _LocaleC_isalpha(int _C)
  {
    return (   _LocaleC_islower(_C)
            || _LocaleC_isupper(_C));
  }

  #pragma inline
  int _LocaleC_iscntrl(int _C)
  {
    return (   (_C>='\x00' && _C<='\x1f')
            || _C=='\x7f');
  }

  #pragma inline
  int _LocaleC_ispunct(int _C)
  {
    return (   (_C>='\x21' && _C<='\x2f')
            || (_C>='\x3a' && _C<='\x40')
            || (_C>='\x5b' && _C<='\x60')
            || (_C>='\x7b' && _C<='\x7e'));
  }

  #pragma inline
  int _LocaleC_isspace(int _C)
  {
    return (   (_C>='\x09' && _C<='\x0d')
            || (_C==' '));
  }

  #pragma inline
  int _LocaleC_tolower(int _C)
  {
    return (_LocaleC_isupper(_C)?_C-'A'+'a':_C);
  }

  #pragma inline
  int _LocaleC_toupper(int _C)
  {
    return (_LocaleC_islower(_C)?_C-'a'+'A':_C);
  }

#endif /* _NO_DEFINITIONS_IN_HEADER_FILES */
_C_STD_END

#endif /* _LOCALE_C_H */
