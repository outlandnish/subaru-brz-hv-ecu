/*
 * Default (weak) implementation of parameter change callback
 * User can override this in their own code
 */
#include "params.h"

// Weak attribute allows user code to override this function
__attribute__((weak)) void Param::Change(Param::PARAM_NUM ParamNum)
{
    // Default implementation does nothing
    // Override this in your application to respond to parameter changes
    (void)ParamNum;
}
