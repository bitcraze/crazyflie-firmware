/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (c) 2012 Bitcraze AB
 *
 * eprintf.c: Memory-friendly ultra-limited embedded implementation of printf
 *
 * This implementation trades CPU cycle for data memory eficiency
 * (No malloc, no unecessary buffers, etc...)
 *
 * Functionality: Implements %s, %d, %x, %X and %f without print size settings.
 * The float handling is verry limited and without exponential notation (ie.
 * works good for number around 0 and within int32 value range).
 *
 * To use this printf a 'putc' function shall be implemented with the prototype
 * 'int putc(int)'. Then a macro calling eprintf can be created. For example:
 * int consolePutc(int c);
 * #define consolePrintf(FMT, ...) eprintf(consolePutc, FMT, ## __VA_ARGS__)
 */
#include "eprintf.h"

#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>

static const char digit[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
                             'A', 'B', 'C', 'D', 'E', 'F'};

int get_int_len (int value, int base)
{
  int l=1;
  while(value>(base - 1))
  {
    l++;
    value/=base;
  }
  return l;
}

int power(int a, int b)
{
  int i;
  int x = a;

  for (i = 1; i < b; i++)
  {
    x *= a;
  }

  return x;
}

static int itoa(putc_t putcf, int num, int base, int precision, int width, char padChar)
{
  long long int i = 1;
  int len = 0;
  unsigned int n = num;
  int numLenght = get_int_len(num, base);

  if (num == 0)
  {
    putcf('0');
    return 1;
  }

  if (num < 0  && base == 10)
  {
    n = -num;
    putcf('-');
  }

  if (numLenght < width)
  {
    int fill = width - numLenght;
    while (fill > 0)
    {
      putcf(padChar);
      len++;
      fill--;
    }
  }

  if (numLenght < precision)
  {
    int fillWithZero = precision -numLenght;
    while (fillWithZero>0)
    {
      putcf('0');
      len++;
      fillWithZero--;
    }
  }

  while (n / i)
  i*=base;

  while (i /= base)
  {
    putcf(digit[(n / i) % base]);
    len++;
  }
  
  return len;
}

int evprintf(putc_t putcf, char * fmt, va_list ap)
{
  int len=0;
  float num;
  char* str;
  int precision;
  int width;
  char padChar;


  while (*fmt)
  {
    if (*fmt == '%')
    {
      precision = 6;
      padChar = ' ';
      width = 0;

      fmt++;
      while ('0' == *fmt)
      {
        padChar = '0';
        fmt++;
      }

			while(isdigit((unsigned)*fmt))
			{
				width *= 10;
				width += *fmt - '0';
				fmt++;
			}

      while (!isalpha((unsigned) *fmt))
      {
        if (*fmt == '.')
        {
          fmt++;
          if (isdigit((unsigned)*fmt))
            precision = *fmt - '0';
            fmt++;
        }
      }
      switch (*fmt++)
      {
        case 'i':
        case 'd':
          len += itoa(putcf, va_arg(ap, int), 10 , 0, 0, ' ');
          break;
        case 'u':
          len += itoa(putcf, va_arg(ap, unsigned int), 10 , 0, 0, ' ');
          break;
        case 'l':
          if (*fmt++ == 'u')
            len += itoa(putcf, va_arg(ap, long unsigned int), 10 , 0, 0, ' ');
          break;
        case 'x':
        case 'X':
          len += itoa(putcf, va_arg(ap, int), 16 , 0, width, padChar);
          break;
        case 'f':
          num = va_arg(ap, double);
          if(num<0)
          {
            putcf('-');
            num = -num;
            len++;
          }
          len += itoa(putcf, (int)num, 10, 0, 0, ' ');
          putcf('.'); len++;
          len += itoa(putcf, (num - (int)num) * power(10,precision), 10, precision, 0, ' ');
          break;
        case 's':
          str = va_arg( ap, char* );
          while(*str)
          {
            putcf(*str++);
            len++;
          }
          break;
        default:
          break;
      }
    }
    else
    {
      putcf(*fmt++);
      len++;
    }
  }
  
  return len;
}

int eprintf(putc_t putcf, char * fmt, ...)
{
  va_list ap;
  int len;

  va_start(ap, fmt);
  len = evprintf(putcf, fmt, ap);
  va_end(ap);

  return len;
}
