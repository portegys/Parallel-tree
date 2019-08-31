/*
 * This software is provided under the terms of the GNU General
 * Public License as published by the Free Software Foundation.
 *
 * Copyright (c) 2003 Tom Portegys, All Rights Reserved.
 * Permission to use, copy, modify, and distribute this software
 * and its documentation for NON-COMMERCIAL purposes and without
 * fee is hereby granted provided that this copyright notice
 * appears in all copies.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.
 */

// Common definitions.

#ifndef __COMMON_H__
#define __COMMON_H__

// Operations.
#define NOP                  (-1)
#define INIT                 0
#define AIM                  1
#define MOVE                 2
#define BALANCE              3
#define MIGRATE              4
#define READY                5
#define REPORT               6
#define REPORT_RESULT        7
#define VIEW                 8
#define VIEW_RESULT          9
#define INSERT               10
#define SEARCH               11
#define SEARCH_RESULT        12
#define STATS                13
#define STATS_RESULT         14
#define QUIT                 15

// Maximum items per message.
#define MAX_MESSAGE_ITEMS    20
#endif
