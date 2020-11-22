#ifndef C_WORHP_IO_H
#define C_WORHP_IO_H

#include "worhp_macros.h"

enum {
  WORHP_PRINT_MESSAGE = 1,
  WORHP_PRINT_WARNING = 2,
  WORHP_PRINT_ERROR = 4
};

typedef void (*worhp_print_t) (int mode, const char s[]);

/**
 * Print function used internally by WORHP. Call it to print
 * your message the same way WORHP currently would.
 *
 * @note WorhpPrint is a wrapper around the actual print function
 * to ensure the function-pointer is non-null.
 * Do not pass WorhpPrint to SetWorhpPrint, since this will cause
 * an infinite recursion error (i.e. hangs or segfaults).
 * SetWorhpPrint detects this and falls back to the default print
 * function.
 */
DLL_PUBLIC void WorhpPrint(const int mode, const char message[]);

/**
 * The actual function that does the printing by default.
 * Pass this function to SetWorhpPrint to restore the default
 * printing behaviour.
 */
DLL_PUBLIC void WorhpDefaultPrintFunction(int mode, const char *message);

/**
 * Function for defining the low-level print function to be used by WORHP
 */
DLL_PUBLIC void SetWorhpPrint(worhp_print_t f);

/**
 * Prints an informative message including its origin, or continues a message.
 * @note Fortran-independent C implementation, since string interoperability
 * is inconvenient.
 * @see WorhpMessage
 */
DLL_PUBLIC void WorhpMessage(const char *message, const char *source, int prn);


/**
 * Prints a warning message including its origin, or continues a warning
 * message.
 * @note Fortran-independent C implementation, since string interoperability
 * is inconvenient.
 * @see WorhpError
 */
DLL_PUBLIC void WorhpWarning(const char *message, const char *source, int prn);


/**
 * Prints an error message including its origin, or continues an error
 * message.
 * @note Fortran-independent C implementation, since string interoperability
 * is inconvenient.
 * @see WorhpError
 */
DLL_PUBLIC void WorhpError(const char *message, const char *source, int prn);

#endif
