/* MSP430-specific DLib configuration. */
/* Copyright © (C) 2004-2008 IAR Systems AB. */

#ifndef _DLIB_PRODUCT_H
#define _DLIB_PRODUCT_H

/*
 * This is the MSP430-specific configuration file for DLib.
 *
 * This file is included right after the _DLIB_CONFIG_FILE (like
 * DLib_Config.h) file that the user can use to configure the library.
 * The file DLib_Defaults.h is then included to set up defaults for
 * all configuration variables that haven't got a value.
 */

/* Special placement for locale structures when building ropi libraries */
#if defined(__ROPI__)
#define _LOCALE_PLACEMENT_ static
#endif

#endif /* _DLIB_PRODUCT_H */
