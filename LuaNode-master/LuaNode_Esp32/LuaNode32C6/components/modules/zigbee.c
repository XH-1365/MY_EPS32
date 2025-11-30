#include <stdio.h>
//#include "modules.h"
#include "lauxlib.h"
#include "ldebug.h"
#include "ldo.h"
#include "lfunc.h"
#include "lmem.h"
#include "lobject.h"
#include "lstate.h"
#include "lualib.h"

#include "lopcodes.h"
#include "lstring.h"
#include "lundump.h"

#include "platform.h"
#include "lrotable.h"
#include "lrodefs.h"

#define TAG	"zb"

// Module function map
const LUA_REG_TYPE zigbee_map[] =
{
	{ LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_zigbee(lua_State *L)
{
	luaL_register( L, LUA_ZIGBEELIBNAME, zigbee_map );
	return 1;
}
